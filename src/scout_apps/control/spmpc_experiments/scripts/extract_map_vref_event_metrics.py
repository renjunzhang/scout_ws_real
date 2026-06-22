#!/usr/bin/env python3
"""Extract PRE/EVENT/POST Map-vref smoke metrics from recorder CSV files.

This is an offline helper for Map-vref software-chain smoke tests. It reads the
lightweight recorder CSV produced during S0/S1 runs plus the frozen/profile CSV
used by spmpc_local_planner. It does not start ROS/Gazebo and does not infer
slosh or RGB conclusions.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


NONE_ZONES = {"", "NONE", "BASE", "BASELINE"}
FLOAT_COLUMNS = ("elapsed", "progress_s", "v_ref_current", "cmd_x", "odom_vx", "cmd_w", "odom_wz")
SLOSH_PROXY_ALIASES = ("slosh_proxy_mm", "slosh_height_mm", "slosh_height", "spmpc_slosh_height", "liquid_height")


def nan() -> float:
    return float("nan")


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def parse_float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return nan()


def fmt(value: Any, digits: int = 3) -> str:
    if isinstance(value, float):
        if math.isfinite(value):
            return f"{value:.{digits}f}"
        return "nan"
    return str(value)


def finite_values(values: Iterable[float]) -> List[float]:
    return [v for v in values if finite(v)]


def safe_mean(values: Iterable[float]) -> float:
    vals = finite_values(values)
    return mean(vals) if vals else nan()


def percentile(values: Iterable[float], pct: float) -> float:
    vals = sorted(finite_values(values))
    if not vals:
        return nan()
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * pct / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    ratio = pos - lo
    return vals[lo] * (1.0 - ratio) + vals[hi] * ratio


def safe_min(values: Iterable[float]) -> float:
    vals = finite_values(values)
    return min(vals) if vals else nan()


def safe_max(values: Iterable[float]) -> float:
    vals = finite_values(values)
    return max(vals) if vals else nan()


def read_samples(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            row: Dict[str, Any] = dict(raw)
            for col in FLOAT_COLUMNS:
                row[col] = parse_float(raw.get(col))
            row["slosh_proxy"] = nan()
            for alias in SLOSH_PROXY_ALIASES:
                value = parse_float(raw.get(alias))
                if finite(value):
                    row["slosh_proxy"] = value
                    break
            rows.append(row)
    rows.sort(key=lambda r: r["elapsed"] if finite(r.get("elapsed")) else math.inf)
    return rows


def find_column(header: Sequence[str], aliases: Sequence[str]) -> Optional[str]:
    lowered = {name.strip().lower(): name for name in header}
    for alias in aliases:
        if alias.lower() in lowered:
            return lowered[alias.lower()]
    return None


def read_profile_zones(path: Path, requested_zones: Optional[Sequence[str]] = None) -> Tuple[List[Dict[str, Any]], float]:
    requested = {z.upper() for z in requested_zones} if requested_zones else None
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        header = reader.fieldnames or []
        s_col = find_column(header, ("s_m", "s", "progress_s_m"))
        v_col = find_column(header, ("v_ref_map_mps", "v_ref_current_mps", "v_ref_mps", "v_safe_mps"))
        if not s_col or not v_col:
            raise SystemExit(f"Profile CSV must contain s and v_ref columns: {path}")
        raw_rows = []
        for raw in reader:
            s_m = parse_float(raw.get(s_col))
            v_ref = parse_float(raw.get(v_col))
            zone = str(raw.get("zone", "") or "").strip().upper()
            raw_rows.append({"raw": raw, "s_m": s_m, "v_ref": v_ref, "zone": zone})

    base_candidates = [r["v_ref"] for r in raw_rows if r["zone"] in NONE_ZONES and finite(r["v_ref"])]
    base_v_ref = base_candidates[0] if base_candidates else nan()

    zones: List[Dict[str, Any]] = []
    current: Optional[Dict[str, Any]] = None
    for row in raw_rows:
        zone = row["zone"]
        if zone in NONE_ZONES or not finite(row["s_m"]) or not finite(row["v_ref"]):
            if current is not None:
                zones.append(current)
                current = None
            continue
        if requested is not None and zone not in requested:
            if current is not None:
                zones.append(current)
                current = None
            continue
        event_id = str(row["raw"].get("event_id", "") or "")
        should_start = (
            current is None
            or current["zone"] != zone
            or current["event_id"] != event_id
            or abs(current["v_ref_map_mps"] - row["v_ref"]) > 1e-9
        )
        if should_start:
            if current is not None:
                zones.append(current)
            current = {
                "zone": zone,
                "event_id": event_id,
                "v_ref_map_mps": row["v_ref"],
                "profile_s_start_m": row["s_m"],
                "profile_s_last_m": row["s_m"],
                "profile_sample_count": 1,
            }
        else:
            current["profile_s_last_m"] = row["s_m"]
            current["profile_sample_count"] += 1
    if current is not None:
        zones.append(current)
    return zones, base_v_ref


def active_samples(rows: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    active = []
    for row in rows:
        status = str(row.get("spmpc_status", ""))
        if "GOAL_REACHED" in status:
            continue
        if finite(row.get("elapsed")) and finite(row.get("v_ref_current")):
            active.append(row)
    return active


def contiguous_indices_matching(
    rows: Sequence[Dict[str, Any]],
    start_idx: int,
    target: float,
    target_tol: float,
) -> List[int]:
    indices: List[int] = []
    in_group = False
    for idx in range(start_idx, len(rows)):
        v_ref = rows[idx].get("v_ref_current")
        match = finite(v_ref) and abs(float(v_ref) - target) <= target_tol
        if match:
            indices.append(idx)
            in_group = True
        elif in_group:
            break
    return indices


def last_contiguous_previous_target(
    rows: Sequence[Dict[str, Any]],
    before_idx: int,
    target: float,
    target_tol: float,
) -> List[int]:
    indices: List[int] = []
    for idx in range(before_idx - 1, -1, -1):
        v_ref = rows[idx].get("v_ref_current")
        if finite(v_ref) and abs(float(v_ref) - target) <= target_tol:
            indices.append(idx)
        elif indices:
            break
    return list(reversed(indices))


def find_transition_start(
    rows: Sequence[Dict[str, Any]],
    start_idx: int,
    previous_target: float,
    target: float,
    change_tol: float,
) -> int:
    for idx in range(start_idx, len(rows)):
        v_ref = rows[idx].get("v_ref_current")
        if not finite(v_ref):
            continue
        moved_from_previous = abs(float(v_ref) - previous_target) > change_tol
        closer_to_target = abs(float(v_ref) - target) < abs(previous_target - target)
        if moved_from_previous and closer_to_target:
            return idx
    return start_idx


def response_delay(
    rows: Sequence[Dict[str, Any]],
    start_idx: int,
    end_idx: int,
    signal_key: str,
    previous_mean: float,
    current_mean: float,
) -> float:
    if not finite(previous_mean) or not finite(current_mean):
        return nan()
    if abs(previous_mean - current_mean) < 1e-9:
        return 0.0
    threshold = 0.5 * (previous_mean + current_mean)
    decreasing = current_mean < previous_mean
    start_elapsed = rows[start_idx].get("elapsed")
    if not finite(start_elapsed):
        return nan()
    for idx in range(start_idx, min(end_idx + 1, len(rows))):
        value = rows[idx].get(signal_key)
        elapsed = rows[idx].get("elapsed")
        if not finite(value) or not finite(elapsed):
            continue
        if (decreasing and value <= threshold) or ((not decreasing) and value >= threshold):
            return float(elapsed) - float(start_elapsed)
    return nan()


def row_metrics(rows: Sequence[Dict[str, Any]]) -> Dict[str, float]:
    if not rows:
        return {
            "row_count": 0,
            "elapsed_start_s": nan(),
            "elapsed_end_s": nan(),
            "duration_s": nan(),
            "progress_start": nan(),
            "progress_end": nan(),
            "v_ref_mean_mps": nan(),
            "v_ref_min_mps": nan(),
            "v_ref_max_mps": nan(),
            "cmd_x_mean_mps": nan(),
            "cmd_x_p95_mps": nan(),
            "cmd_x_max_mps": nan(),
            "odom_vx_mean_mps": nan(),
            "odom_vx_p95_mps": nan(),
            "odom_vx_max_mps": nan(),
            "slosh_proxy_mean": nan(),
            "slosh_proxy_p95": nan(),
            "slosh_proxy_max": nan(),
        }
    elapsed_start = rows[0]["elapsed"]
    elapsed_end = rows[-1]["elapsed"]
    return {
        "row_count": len(rows),
        "elapsed_start_s": elapsed_start,
        "elapsed_end_s": elapsed_end,
        "duration_s": elapsed_end - elapsed_start if finite(elapsed_start) and finite(elapsed_end) else nan(),
        "progress_start": rows[0]["progress_s"],
        "progress_end": rows[-1]["progress_s"],
        "v_ref_mean_mps": safe_mean(r["v_ref_current"] for r in rows),
        "v_ref_min_mps": safe_min(r["v_ref_current"] for r in rows),
        "v_ref_max_mps": safe_max(r["v_ref_current"] for r in rows),
        "cmd_x_mean_mps": safe_mean(r["cmd_x"] for r in rows),
        "cmd_x_p95_mps": percentile((r["cmd_x"] for r in rows), 95.0),
        "cmd_x_max_mps": safe_max(r["cmd_x"] for r in rows),
        "odom_vx_mean_mps": safe_mean(r["odom_vx"] for r in rows),
        "odom_vx_p95_mps": percentile((r["odom_vx"] for r in rows), 95.0),
        "odom_vx_max_mps": safe_max(r["odom_vx"] for r in rows),
        "slosh_proxy_mean": safe_mean(r["slosh_proxy"] for r in rows),
        "slosh_proxy_p95": percentile((r["slosh_proxy"] for r in rows), 95.0),
        "slosh_proxy_max": safe_max(r["slosh_proxy"] for r in rows),
    }


def extract_metrics(args: argparse.Namespace) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    samples = read_samples(args.samples)
    zones, base_v_ref = read_profile_zones(args.profile, args.zones)
    rows = active_samples(samples)
    if not rows:
        raise SystemExit(f"No active samples found in {args.samples}")
    if not zones:
        raise SystemExit(f"No non-NONE profile zones found in {args.profile}")

    summary = {
        "case_id": args.case_id,
        "samples_csv": str(args.samples),
        "profile_csv": str(args.profile),
        "sample_count": len(samples),
        "active_sample_count": len(rows),
        "duration_s": samples[-1]["elapsed"] if samples else nan(),
        "final_status": str(samples[-1].get("spmpc_status", "")) if samples else "",
        "final_progress": samples[-1].get("progress_s", nan()) if samples else nan(),
        "map_vref_statuses": sorted({str(r.get("map_vref_status", "")) for r in samples}),
        "active_v_ref_min_mps": safe_min(r["v_ref_current"] for r in rows),
        "active_v_ref_max_mps": safe_max(r["v_ref_current"] for r in rows),
        "base_v_ref_mps": base_v_ref,
        "target_tolerance_mps": args.target_tolerance,
        "change_tolerance_mps": args.change_tolerance,
        "notes": "offline Map-vref smoke metrics only; not RGB/slosh formal evidence",
    }

    metrics: List[Dict[str, Any]] = []
    start_idx = 0
    previous_target = base_v_ref if finite(base_v_ref) else zones[0]["v_ref_map_mps"]
    for zone in zones:
        target = float(zone["v_ref_map_mps"])
        transition_idx = find_transition_start(rows, start_idx, previous_target, target, args.change_tolerance)
        plateau_indices = contiguous_indices_matching(rows, transition_idx, target, args.target_tolerance)
        if not plateau_indices:
            plateau_indices = contiguous_indices_matching(rows, start_idx, target, args.target_tolerance)
        plateau_rows = [rows[idx] for idx in plateau_indices]
        previous_indices = last_contiguous_previous_target(rows, transition_idx, previous_target, args.target_tolerance)
        previous_rows = [rows[idx] for idx in previous_indices]
        metric: Dict[str, Any] = dict(zone)
        metric.update(row_metrics(plateau_rows))
        metric["case_id"] = args.case_id
        metric["target_v_ref_mps"] = target
        metric["previous_target_v_ref_mps"] = previous_target
        metric["transition_start_elapsed_s"] = rows[transition_idx]["elapsed"] if rows else nan()
        metric["transition_start_progress"] = rows[transition_idx]["progress_s"] if rows else nan()
        if len(previous_rows) >= 4:
            previous_steady_rows = previous_rows[len(previous_rows) // 2 :]
        else:
            previous_steady_rows = previous_rows
        metric["previous_row_count"] = len(previous_rows)
        metric["previous_cmd_x_mean_mps"] = safe_mean(r["cmd_x"] for r in previous_steady_rows)
        metric["previous_odom_vx_mean_mps"] = safe_mean(r["odom_vx"] for r in previous_steady_rows)
        if plateau_indices:
            first_idx = plateau_indices[0]
            last_idx = plateau_indices[-1]
            metric["cmd_x_half_step_response_delay_s"] = response_delay(
                rows,
                transition_idx,
                last_idx,
                "cmd_x",
                metric["previous_cmd_x_mean_mps"],
                metric["cmd_x_mean_mps"],
            )
            metric["odom_vx_half_step_response_delay_s"] = response_delay(
                rows,
                transition_idx,
                last_idx,
                "odom_vx",
                metric["previous_odom_vx_mean_mps"],
                metric["odom_vx_mean_mps"],
            )
            start_idx = last_idx + 1
        else:
            first_idx = transition_idx
            metric["cmd_x_half_step_response_delay_s"] = nan()
            metric["odom_vx_half_step_response_delay_s"] = nan()
            start_idx = transition_idx + 1
        metric["plateau_found"] = bool(plateau_indices)
        metric["plateau_start_index"] = first_idx
        metric["plateau_end_index"] = plateau_indices[-1] if plateau_indices else first_idx
        metrics.append(metric)
        previous_target = target
    return summary, metrics


def write_csv(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    preferred = [
        "case_id",
        "zone",
        "event_id",
        "profile_s_start_m",
        "profile_s_last_m",
        "target_v_ref_mps",
        "previous_target_v_ref_mps",
        "transition_start_elapsed_s",
        "transition_start_progress",
        "elapsed_start_s",
        "elapsed_end_s",
        "duration_s",
        "progress_start",
        "progress_end",
        "row_count",
        "v_ref_mean_mps",
        "cmd_x_mean_mps",
        "odom_vx_mean_mps",
        "previous_cmd_x_mean_mps",
        "previous_odom_vx_mean_mps",
        "cmd_x_half_step_response_delay_s",
        "odom_vx_half_step_response_delay_s",
        "slosh_proxy_mean",
        "slosh_proxy_p95",
        "slosh_proxy_max",
        "plateau_found",
    ]
    extra = sorted({k for row in rows for k in row.keys() if k not in preferred})
    fieldnames = preferred + extra
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def markdown_table(rows: Sequence[Dict[str, Any]]) -> str:
    lines = [
        "| zone | profile s m | progress_s | v_ref mean | cmd_x mean | odom_vx mean | slosh proxy p95 | cmd delay s | odom delay s |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {zone} | {s0}~{s1} | {p0}~{p1} | {v} | {cmd} | {odom} | {slosh} | {cmd_delay} | {odom_delay} |".format(
                zone=row.get("zone", ""),
                s0=fmt(row.get("profile_s_start_m")),
                s1=fmt(row.get("profile_s_last_m")),
                p0=fmt(row.get("progress_start")),
                p1=fmt(row.get("progress_end")),
                v=fmt(row.get("v_ref_mean_mps")),
                cmd=fmt(row.get("cmd_x_mean_mps")),
                odom=fmt(row.get("odom_vx_mean_mps")),
                slosh=fmt(row.get("slosh_proxy_p95")),
                cmd_delay=fmt(row.get("cmd_x_half_step_response_delay_s")),
                odom_delay=fmt(row.get("odom_vx_half_step_response_delay_s")),
            )
        )
    return "\n".join(lines)


def write_markdown(path: Path, summary: Dict[str, Any], rows: Sequence[Dict[str, Any]]) -> None:
    content = f"""# Map-vref event metrics summary

```text
case_id={summary['case_id']}
samples_csv={summary['samples_csv']}
profile_csv={summary['profile_csv']}
final_status={summary['final_status']}
duration_s={fmt(summary['duration_s'])}
final_progress={fmt(summary['final_progress'])}
map_vref_statuses={','.join(summary['map_vref_statuses'])}
active_v_ref_min_mps={fmt(summary['active_v_ref_min_mps'])}
active_v_ref_max_mps={fmt(summary['active_v_ref_max_mps'])}
```

{markdown_table(rows)}

Notes:

```text
1. This is an offline Map-vref smoke metric, not RGB/slosh formal evidence.
2. Response delay is measured from the first v_ref_current transition toward a zone to the first half-step crossing of cmd_x/odom_vx.
3. If only normalized progress_s is available, profile s_m and observed progress_s are reported side by side rather than treated as the same coordinate.
4. slosh proxy fields are NaN unless the recorder CSV includes one of: {', '.join(SLOSH_PROXY_ALIASES)}.
```
"""
    path.write_text(content, encoding="utf-8")


def print_summary(summary: Dict[str, Any], rows: Sequence[Dict[str, Any]]) -> None:
    print(f"case_id={summary['case_id']}")
    print(f"final_status={summary['final_status']}")
    print(f"duration_s={fmt(summary['duration_s'])}")
    print(f"final_progress={fmt(summary['final_progress'])}")
    print(f"map_vref_statuses={','.join(summary['map_vref_statuses'])}")
    for row in rows:
        print(
            "{zone}: profile_s={s0}..{s1} progress={p0}..{p1} "
            "v_ref={v} cmd_x={cmd} odom_vx={odom} cmd_delay={cmd_delay} odom_delay={odom_delay}".format(
                zone=row.get("zone", ""),
                s0=fmt(row.get("profile_s_start_m")),
                s1=fmt(row.get("profile_s_last_m")),
                p0=fmt(row.get("progress_start")),
                p1=fmt(row.get("progress_end")),
                v=fmt(row.get("v_ref_mean_mps")),
                cmd=fmt(row.get("cmd_x_mean_mps")),
                odom=fmt(row.get("odom_vx_mean_mps")),
                cmd_delay=fmt(row.get("cmd_x_half_step_response_delay_s")),
                odom_delay=fmt(row.get("odom_vx_half_step_response_delay_s")),
            )
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=Path, required=True, help="Recorder CSV with progress/v_ref/cmd/odom columns")
    parser.add_argument("--profile", type=Path, required=True, help="Map-vref profile CSV with s_m and v_ref columns")
    parser.add_argument("--case-id", default="map_vref_event_smoke")
    parser.add_argument("--zones", nargs="*", default=None, help="Optional zone names to keep, e.g. PRE EVENT POST")
    parser.add_argument("--target-tolerance", type=float, default=0.006, help="Tolerance for plateau v_ref matching")
    parser.add_argument("--change-tolerance", type=float, default=0.006, help="Tolerance for transition-start detection")
    parser.add_argument("--out-csv", type=Path, help="Write per-zone metrics CSV")
    parser.add_argument("--out-md", type=Path, help="Write Markdown summary")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary, metrics = extract_metrics(args)
    if args.out_csv:
        args.out_csv.parent.mkdir(parents=True, exist_ok=True)
        write_csv(args.out_csv, metrics)
    if args.out_md:
        args.out_md.parent.mkdir(parents=True, exist_ok=True)
        write_markdown(args.out_md, summary, metrics)
    print_summary(summary, metrics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
