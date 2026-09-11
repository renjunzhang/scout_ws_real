#!/usr/bin/env python3
"""Offline metrics for one RGB-free internal-slosh recording.

The single-recording path reads exactly six small topics and caches their
scalarized messages.  The comparison path preserves every supplied report and
only applies a primary-monitor/evaluation lock when one is explicitly given.
No RGB topic is read and no monitor is selected independently per recording.
"""

import argparse
import hashlib
import importlib.util
import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np


TOPICS = {
    "/spmpc/debug/control_cycle_audit": "audit",
    "/spmpc/debug/pre_solve_snapshot": "snapshot",
    "/spmpc/debug/slosh_observer_imu": "imu",
    "/spmpc/debug/slosh_observer_odom": "odom",
    "/spmpc/debug/effective_config": "config",
    "/scout/global_path_fixed": "path",
}
MONITORS = ("imu", "odom")
WINDOW_NAMES = ("pre1s", "task", "path10_90", "goal_post5s")
POSTFLIGHT_SUFFIXES = (
    "recording_postflight",
    "runtime_postflight",
    "ablation_postflight",
    "i0_explicit_actuator_contract_postflight",
)
PROTOCOL = "SMPCC_C03_INTERNAL_SLOSH_DEV_V2"
MONITOR_MAX_GAP_SEC = {"imu": 0.035, "odom": 0.050}
BSLOSH_WEIGHT_KEYS = (
    "/spmpc_local_planner/variants/B_slosh/w_slosh",
    "variants/B_slosh/w_slosh",
)
EVALUATION_SPEC = {
    "protocol": PROTOCOL,
    "topics": tuple(TOPICS),
    "monitors": MONITORS,
    "windows": ("task", "path10_90", "goal_post5s"),
    "metrics": ("rms_mm", "peak_mm", "p95_mm", "peak_stamp_sec"),
}


def _get(value: Any, name: str, default: Any = None) -> Any:
    if isinstance(value, Mapping):
        return value.get(name, default)
    return getattr(value, name, default)


def _stamp(value: Any) -> Optional[float]:
    if value is None:
        return None
    if hasattr(value, "to_sec"):
        value = value.to_sec()
    elif hasattr(value, "secs") and hasattr(value, "nsecs"):
        value = float(value.secs) + 1.0e-9 * float(value.nsecs)
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _primitive(value: Any) -> Any:
    stamp = _stamp(value)
    if stamp is not None and not isinstance(value, (str, bytes, bool, int, float)):
        return stamp
    if isinstance(value, (bool, int, float, str)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [_primitive(item) for item in value]
    return None


def scalar_message(message: Any) -> Dict[str, Any]:
    """Copy scalar ROS fields, converting ROS times to seconds."""
    result: Dict[str, Any] = {}
    names = getattr(message, "__slots__", None)
    if names is None and isinstance(message, Mapping):
        names = list(message.keys())
    for name in names or ():
        value = _get(message, name)
        if name == "header":
            result["header_stamp"] = _stamp(_get(value, "stamp"))
            result["frame_id"] = _get(value, "frame_id", "")
            continue
        converted = _primitive(value)
        if converted is not None:
            result[name] = converted
    return result


def _config_message(message: Any) -> Dict[str, Any]:
    layout = _get(message, "layout")
    dimensions = _get(layout, "dim", []) or []
    label = _get(dimensions[0], "label", "") if dimensions else ""
    data = list(_get(message, "data", []) or [])
    names = [item.strip() for item in str(label).split(",") if item.strip()]
    return dict(zip(names, [float(value) for value in data]))


def _path_message(message: Any) -> Dict[str, Any]:
    poses = []
    for pose_stamped in _get(message, "poses", []) or []:
        pose = _get(pose_stamped, "pose", pose_stamped)
        position = _get(pose, "position")
        poses.append([float(_get(position, "x")), float(_get(position, "y"))])
    return {
        "frame_id": _get(_get(message, "header"), "frame_id", ""),
        "xy": poses,
    }


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_env(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    if not path.is_file():
        return values
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def extract_topics(path: Path) -> Dict[str, Any]:
    """Read only TOPICS from one bag; no RGB or image topic is requested."""
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("rosbag is unavailable: {}".format(exc))
    extracted: Dict[str, Any] = {name: [] for name in TOPICS.values()}
    with rosbag.Bag(str(path), "r") as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())
        missing = sorted(set(TOPICS) - available)
        if missing:
            raise RuntimeError("missing required topics: {}".format(missing))
        for topic, message, bag_stamp in bag.read_messages(topics=list(TOPICS)):
            name = TOPICS[topic]
            receive_stamp = _stamp(bag_stamp)
            if name == "config":
                row = _config_message(message)
            elif name == "path":
                row = _path_message(message)
                if extracted[name] and row == extracted[name][-1]["value"]:
                    continue
            else:
                row = scalar_message(message)
            extracted[name].append({"receive_stamp": receive_stamp, "value": row})
    return extracted


def load_cached_topics(
    bag: Path, cache: Path, *, extractor=extract_topics
) -> Tuple[Dict[str, Any], bool]:
    identity = {
        "bag": str(bag),
        "size": bag.stat().st_size,
        "mtime_ns": bag.stat().st_mtime_ns,
        "topics": sorted(TOPICS),
    }
    if cache.is_file():
        payload = json.loads(cache.read_text(encoding="utf-8"))
        if payload.get("identity") == identity:
            return payload["topics"], True
    topics = extractor(bag)
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(json.dumps({"identity": identity, "topics": topics}, indent=2) + "\n")
    return topics, False


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _number(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _env_zero(values: Mapping[str, str], key: str) -> bool:
    return key in values and _finite(values.get(key)) and float(values[key]) == 0.0


@lru_cache(maxsize=1)
def _current_evaluation_chain_sha() -> Optional[str]:
    """Return the frozen-chain digest used by the companion lock helper."""
    helper_path = Path(__file__).with_name("freeze_internal_slosh_evaluation.py")
    if not helper_path.is_file():
        return None
    spec = importlib.util.spec_from_file_location("_internal_slosh_freeze", helper_path)
    if spec is None or spec.loader is None:
        return None
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    return helper.chain_identity()["sha256"]


def _audit_stamp(row: Mapping[str, Any], name: str) -> Optional[float]:
    return _stamp(row.get(name))


def _motion_window(audits: Sequence[Mapping[str, Any]]) -> Tuple[float, float]:
    published = [row for row in audits if row.get("command_was_published")]
    start = next(
        (
            _audit_stamp(row, "command_publish_stamp")
            for row in published
            if _finite(_audit_stamp(row, "command_publish_stamp"))
            and max(abs(_number(row.get("published_cmd_v"))), abs(_number(row.get("published_cmd_omega")))) > 1e-9
        ),
        None,
    )
    if start is None:
        raise ValueError("first nonzero command_publish_stamp is missing")
    goal = next(
        (
            _audit_stamp(row, "command_publish_stamp")
            for row in published
            if _audit_stamp(row, "command_publish_stamp") is not None
            and (_get(row, "status", "") == "GOAL_REACHED" or _get(row, "solver_status", "") == "GOAL_REACHED")
            and _audit_stamp(row, "command_publish_stamp") >= start
        ),
        None,
    )
    if goal is None or goal <= start:
        raise ValueError("first GOAL_REACHED command_publish_stamp is missing")
    return float(start), float(goal)


def project_polyline(xy: np.ndarray, path: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    if len(path) < 2:
        raise ValueError("global path has fewer than two points")
    segments = np.diff(path, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    if np.any(lengths <= 1e-12):
        raise ValueError("global path contains zero-length segment")
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    offset = xy[:, None, :] - path[:-1][None, :, :]
    ratio = np.clip(np.sum(offset * segments[None, :, :], axis=2) / (lengths * lengths), 0.0, 1.0)
    delta = offset - ratio[:, :, None] * segments[None, :, :]
    distance_sq = np.sum(delta * delta, axis=2)
    index = np.argmin(distance_sq, axis=1)
    progress = ratio[np.arange(len(xy)), index] * lengths[index] + cumulative[index]
    return progress, np.sqrt(distance_sq[np.arange(len(xy)), index]), float(cumulative[-1])


def first_crossing(time: np.ndarray, progress: np.ndarray, target: float) -> float:
    crossings = np.flatnonzero((progress[:-1] <= target) & (progress[1:] >= target))
    if len(crossings) == 0:
        raise ValueError("path does not cross {:.6g}".format(target))
    index = int(crossings[0])
    delta = progress[index + 1] - progress[index]
    if abs(delta) <= 1e-12:
        return float(time[index])
    return float(time[index] + (target - progress[index]) * (time[index + 1] - time[index]) / delta)


def monitor_stats(
    rows: Sequence[Mapping[str, Any]], lo: float, hi: float, task_start: float,
    *, max_gap_sec: float,
) -> Dict[str, Any]:
    candidates = []
    invalid = 0
    for row in rows:
        stamp = _stamp(row.get("state_stamp"))
        if stamp is None or stamp < lo or stamp > hi:
            continue
        valid = bool(row.get("valid")) and bool(row.get("configured")) and _finite(row.get("modal_height_m"))
        if not valid:
            invalid += 1
            continue
        candidates.append((stamp, float(row["modal_height_m"]) * 1000.0))
    # Preserve bag order for the continuity check, then sort only for the
    # order-independent summary statistics and peak time.
    duplicate_or_regress = bool(
        len(candidates) > 1 and np.any(np.diff(np.asarray([item[0] for item in candidates])) <= 0.0)
    )
    candidates.sort(key=lambda item: item[0])
    stamps = np.asarray([item[0] for item in candidates], dtype=float)
    heights = np.asarray([item[1] for item in candidates], dtype=float)
    if len(heights) == 0:
        return {
            "n": 0, "raw_n": invalid, "invalid_state_count": invalid,
            "valid_fraction": 0.0, "continuous_valid": False,
            "coverage_valid": False, "interval_valid": False,
            "status": "FAIL",
        }
    peak_index = int(np.argmax(heights))
    gaps = np.diff(stamps)
    coverage_valid = bool(
        len(stamps) >= 2
        and stamps[0] <= lo + max_gap_sec
        and stamps[-1] >= hi - max_gap_sec
    )
    interval_valid = bool(len(gaps) == 0 or np.all(gaps <= max_gap_sec + 1.0e-9))
    continuous_valid = bool(
        len(stamps) >= 2 and invalid == 0 and not duplicate_or_regress
        and coverage_valid and interval_valid
    )
    return {
        "n": int(len(heights)),
        "raw_n": int(len(heights) + invalid),
        "invalid_state_count": int(invalid),
        "valid_fraction": float(len(heights) / max(1, len(heights) + invalid)),
        "continuous_valid": continuous_valid,
        "coverage_valid": coverage_valid,
        "interval_valid": interval_valid,
        "status": "PASS" if continuous_valid else "FAIL",
        "rms_mm": float(np.sqrt(np.mean(heights * heights))),
        "peak_mm": float(np.max(heights)),
        "p95_mm": float(np.percentile(heights, 95.0)),
        "peak_stamp_sec": float(stamps[peak_index]),
        "peak_time_from_task_start_sec": float(stamps[peak_index] - task_start),
        "max_state_gap_ms": float(np.max(gaps) * 1000.0) if len(gaps) else 0.0,
    }


def _postflights(bag: Path) -> Dict[str, Any]:
    result = {}
    for suffix in POSTFLIGHT_SUFFIXES:
        path = bag.with_name(bag.stem + "_" + suffix + ".json")
        if not path.is_file():
            result[suffix] = None
            continue
        result[suffix] = json.loads(path.read_text(encoding="utf-8"))
    return result


def analyze_topics(
    topics: Mapping[str, Any], bag: Path, *, prereg_path: Optional[Path] = None
) -> Dict[str, Any]:
    """Build the reviewable single-record report from scalarized topics."""
    audits = [entry["value"] for entry in topics.get("audit", [])]
    snapshots = [entry["value"] for entry in topics.get("snapshot", [])]
    windows: Dict[str, List[float]] = {}
    report: Dict[str, Any] = {"bag": str(bag), "status": "FAIL", "eligible_for_comparison": False}
    # Preserve identity and all postflight evidence even when motion/path
    # analysis fails before the numerical sections can be built.
    prereg_file = prereg_path or bag.with_name(bag.stem + "_runtime_smoke_prereg.env")
    report["prereg_file"] = str(prereg_file)
    report["prereg"] = _read_env(prereg_file)
    launch_file = bag.with_name(bag.stem + "_launch_params.yaml")
    report["launch_params_file"] = str(launch_file)
    try:
        report["launch_params_sha256"] = _file_sha256(launch_file) if launch_file.is_file() else None
    except OSError:
        report["launch_params_sha256"] = None
    try:
        report["bag_mtime_epoch_sec"] = bag.stat().st_mtime_ns / 1.0e9 if bag.is_file() else None
    except OSError:
        report["bag_mtime_epoch_sec"] = None
    try:
        postflight_details = _postflights(bag)
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        postflight_details = {suffix: None for suffix in POSTFLIGHT_SUFFIXES}
        report["postflight_error"] = "{}: {}".format(type(exc).__name__, exc)
    report["postflights"] = {
        key: (value.get("status") if value else None)
        for key, value in postflight_details.items()
    }
    report["postflight_details"] = postflight_details
    try:
        start, end = _motion_window(audits)
        windows["pre1s"] = [start - 1.0, start]
        windows["task"] = [start, end]
        windows["goal_post5s"] = [end, end + 5.0]
        path_rows = [entry["value"] for entry in topics.get("path", [])]
        if not path_rows:
            raise ValueError("fixed global path is missing")
        path = np.asarray(path_rows[0].get("xy", []), dtype=float)
        snapshot_rows = [row for row in snapshots if row.get("valid")]
        state_time = np.asarray([_stamp(row.get("robot_state_stamp")) for row in snapshot_rows], dtype=float)
        xy = np.asarray([[float(row["robot_x"]), float(row["robot_y"])] for row in snapshot_rows], dtype=float)
        if len(state_time) < 3 or not np.all(np.isfinite(state_time)):
            raise ValueError("valid snapshots lack finite robot_state_stamp")
        snapshot_time_order_valid = bool(len(state_time) < 2 or np.all(np.diff(state_time) > 0.0))
        ordered = np.argsort(state_time, kind="stable")
        state_time = state_time[ordered]
        xy = xy[ordered]
        progress, error, path_length = project_polyline(xy, path)
        enter = first_crossing(state_time, progress, 0.1 * path_length)
        leave = first_crossing(state_time, progress, 0.9 * path_length)
        windows["path10_90"] = [enter, leave]
        alignment_tolerance = float(np.median(np.diff(state_time))) if len(state_time) > 1 else 0.0
        path_task_intersection = bool(
            enter >= start - alignment_tolerance and leave <= end + alignment_tolerance
        )
        task_mask = (state_time >= start) & (state_time <= end)
        path_mask = (state_time >= enter) & (state_time <= leave)
        if not np.any(task_mask) or not np.any(path_mask):
            raise ValueError("snapshot coverage does not span required windows")
        report["windows"] = windows
        report["task_duration_sec"] = float(end - start)
        report["path_length_m"] = path_length
        report["tracking"] = {
            "task_rms_cm": float(np.sqrt(np.mean((error[task_mask] * 100.0) ** 2))),
            "path10_90_rms_cm": float(np.sqrt(np.mean((error[path_mask] * 100.0) ** 2))),
            "start_offset_cm": float(np.linalg.norm(xy[0] - path[0]) * 100.0),
            "end_offset_cm": float(np.linalg.norm(xy[-1] - path[-1]) * 100.0),
            "initial_snapshot_sec": float(state_time[0]),
            "snapshot_time_order_valid": snapshot_time_order_valid,
            "path10_90_within_task": path_task_intersection,
        }
        report["monitor_height_mm"] = {
            window: {
                monitor: monitor_stats(
                    [entry["value"] for entry in topics.get(monitor, [])],
                    bounds[0], bounds[1], start,
                    max_gap_sec=MONITOR_MAX_GAP_SEC[monitor],
                )
                for monitor in MONITORS
            }
            for window, bounds in windows.items()
            if window in ("pre1s", "task", "path10_90", "goal_post5s")
        }
        report["effective_config_first"] = (
            topics.get("config", [{}])[0].get("value", {}) if topics.get("config") else {}
        )
        config_values = [entry.get("value", {}) for entry in topics.get("config", [])]
        report["effective_config_count"] = len(config_values)
        report["effective_config_consistent"] = bool(config_values) and all(
            _config_equal(config_values[0], value) for value in config_values[1:]
        )
        gates = []
        gates.append((all(report["postflights"].get(key) == "PASS" for key in POSTFLIGHT_SUFFIXES), "postflights"))
        gates.append((bool(report["prereg"]), "prereg"))
        gates.append((report["prereg"].get("protocol") == PROTOCOL, "protocol"))
        gates.append((_env_zero(report["prereg"], "runner_exit_code"), "runner_exit_code"))
        gates.append((_env_zero(report["prereg"], "diagnostics_exit_code"), "diagnostics_exit_code"))
        actual_launch_sha = report["launch_params_sha256"]
        gates.append((actual_launch_sha is not None, "launch_params"))
        gates.append((report["prereg"].get("launch_params_sha256") == actual_launch_sha, "launch_params_sha256"))
        expected_chain = report["prereg"].get("evaluation_chain_sha256")
        current_chain = _current_evaluation_chain_sha()
        gates.append((expected_chain is not None and current_chain is not None and expected_chain == current_chain,
                      "evaluation_chain_sha256"))
        gates.append((report["effective_config_consistent"], "effective_config"))
        gates.append((all(
            report["monitor_height_mm"][window][monitor].get("continuous_valid", False)
            for window in ("task", "path10_90", "goal_post5s") for monitor in MONITORS
        ), "monitor_continuity"))
        gates.append((snapshot_time_order_valid, "snapshot_time_order"))
        gates.append((path_task_intersection, "path10_90_task_intersection"))
        report["gate_failures"] = [name for passed, name in gates if not passed]
        report["status"] = "PASS" if not report["gate_failures"] else "FAIL"
        report["eligible_for_comparison"] = report["status"] == "PASS"
    except Exception as exc:
        report["error"] = "{}: {}".format(type(exc).__name__, exc)
        report["status"] = "FAIL"
    return report


def _config_without_liquid(config: Mapping[str, Any]) -> Dict[str, Any]:
    return {key: value for key, value in config.items() if key not in ("slosh_enable", "w_slosh")}


def _config_equal(left: Any, right: Any) -> bool:
    if isinstance(left, Mapping) and isinstance(right, Mapping):
        return set(left) == set(right) and all(_config_equal(left[key], right[key]) for key in left)
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), rel_tol=1.0e-9, abs_tol=1.0e-9)
    return left == right


def _close_number(left: Any, right: Any, *, rel_tol: float = 1.0e-6, abs_tol: float = 1.0e-6) -> bool:
    return _finite(left) and _finite(right) and math.isclose(
        float(left), float(right), rel_tol=rel_tol, abs_tol=abs_tol
    )


def _report_metric(report: Mapping[str, Any], window: str, monitor: str, metric: str) -> Optional[float]:
    value = report.get("monitor_height_mm", {}).get(window, {}).get(monitor, {}).get(metric)
    return float(value) if _finite(value) else None


def compare_reports(paths: Sequence[Path], output: Path, lock_path: Optional[Path] = None) -> Dict[str, Any]:
    loaded = [(path, json.loads(path.read_text(encoding="utf-8"))) for path in paths]
    result: Dict[str, Any] = {
        "protocol": PROTOCOL,
        "input_reports": [str(path) for path in paths],
        "reports": [report for _, report in loaded],
        "status": "SCREENING" if lock_path is None else "FAIL",
    }
    if lock_path is None:
        result["monitor_comparison"] = {
            path.stem: {
                window: {monitor: report.get("monitor_height_mm", {}).get(window, {}).get(monitor)
                         for monitor in MONITORS}
                for window in ("task", "path10_90", "goal_post5s")
            }
            for path, report in loaded
        }
        result["winner_selected"] = False
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return result
    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    lock_sha = _file_sha256(lock_path)
    failures: List[str] = []
    if lock.get("schema_version") != 1 or lock.get("protocol") != PROTOCOL:
        failures.append("invalid evaluation lock schema/protocol")
    current_chain = _current_evaluation_chain_sha()
    if not lock.get("evaluation_chain_sha256") or current_chain is None or lock.get("evaluation_chain_sha256") != current_chain:
        failures.append("evaluation lock chain SHA does not match current evaluator")
    primary = lock.get("primary_monitor")
    if primary not in MONITORS:
        failures.append("invalid primary_monitor")
    expected_rows = lock.get("rows", {})
    if set(expected_rows) != {"01", "02", "03", "04"}:
        failures.append("lock rows are not 01..04")
    if expected_rows != {"01": "full", "02": "smooth", "03": "smooth", "04": "full"}:
        failures.append("lock rows do not define 01 full / 02 smooth / 03 smooth / 04 full")
    by_row: Dict[str, Dict[str, Any]] = {}
    for report in result["reports"]:
        row = report.get("prereg", {}).get("evaluation_row")
        if row in by_row:
            failures.append("duplicate evaluation row {}".format(row))
        elif row:
            by_row[row] = report
    if set(by_row) != set(expected_rows):
        failures.append("reports do not cover every lock row")
    ordered_reports: List[Dict[str, Any]] = []
    for row in sorted(expected_rows):
        report = by_row.get(row)
        if report is None:
            continue
        ordered_reports.append(report)
        if expected_rows[row] != report.get("prereg", {}).get("condition"):
            failures.append("row {} condition mismatch".format(row))
        if report.get("prereg", {}).get("phase") != "validation":
            failures.append("row {} phase is not validation".format(row))
        if report.get("prereg", {}).get("protocol") != PROTOCOL:
            failures.append("row {} protocol mismatch".format(row))
        if report.get("prereg", {}).get("evaluation_lock_sha256") != lock_sha:
            failures.append("row {} evaluation lock SHA mismatch".format(row))
        if report.get("prereg", {}).get("evaluation_primary_monitor") != primary:
            failures.append("row {} primary monitor mismatch".format(row))
        task_start = report.get("windows", {}).get("task", [None])[0]
        if not _finite(task_start) or float(task_start) <= float(lock.get("created_at_epoch_sec", math.inf)):
            failures.append("row {} task start is not newer than lock".format(row))
        if not report.get("eligible_for_comparison"):
            failures.append("row {} single-report gate failed".format(row))
        launch_hashes = lock.get("launch_params_sha256", {})
        condition = report.get("prereg", {}).get("condition")
        expected_launch_hash = launch_hashes.get(condition) if isinstance(launch_hashes, Mapping) else None
        if expected_launch_hash is None or report.get("launch_params_sha256") != expected_launch_hash:
            failures.append("row {} launch params SHA mismatch".format(row))
        chain_hash = lock.get("evaluation_chain_sha256")
        if not chain_hash or report.get("prereg", {}).get("evaluation_chain_sha256") != chain_hash:
            failures.append("row {} evaluation chain SHA mismatch".format(row))
        for window in ("task", "path10_90", "goal_post5s"):
            if not report.get("monitor_height_mm", {}).get(window, {}).get(primary, {}).get("continuous_valid", False):
                failures.append("row {} primary monitor continuity failed in {}".format(row, window))
    full_weight = lock.get("full_w_slosh")
    smooth_weight = lock.get("smooth_w_slosh")
    if smooth_weight is None:
        smooth_params = lock.get("launch_params", {}).get("smooth", {})
        if isinstance(smooth_params, Mapping):
            smooth_weight = next(
                (smooth_params[key] for key in BSLOSH_WEIGHT_KEYS if key in smooth_params),
                None,
            )
    for row in ("01", "04"):
        report = by_row.get(row)
        if report is not None and not _close_number(report.get("effective_config_first", {}).get("w_slosh"), full_weight):
            failures.append("row {} full w_slosh mismatch".format(row))
    for row in ("02", "03"):
        report = by_row.get(row)
        if report is not None and smooth_weight is not None and not _close_number(
            report.get("effective_config_first", {}).get("w_slosh"), smooth_weight
        ):
            failures.append("row {} smooth w_slosh mismatch".format(row))
    configs = [report.get("effective_config_first", {}) for report in ordered_reports]
    if configs and any(
        not _config_equal(_config_without_liquid(config), _config_without_liquid(configs[0]))
        for config in configs[1:]
    ):
        failures.append("non-liquid effective_config differs across validation rows")
    task_starts = [report.get("windows", {}).get("task", [None])[0] for report in ordered_reports]
    if len(task_starts) == 4 and any(
        not (_finite(left) and _finite(right) and float(left) < float(right))
        for left, right in zip(task_starts, task_starts[1:])
    ):
        failures.append("validation task starts are not strictly increasing")
    pairs = {}
    for full_row, smooth_row in (("01", "02"), ("04", "03")):
        full, smooth = by_row.get(full_row), by_row.get(smooth_row)
        if full is None or smooth is None:
            continue
        pair = {}
        for window in ("task", "path10_90", "goal_post5s"):
            pair[window] = {}
            for metric in ("rms_mm", "peak_mm", "p95_mm"):
                old = _report_metric(full, window, primary, metric)
                new = _report_metric(smooth, window, primary, metric)
                pair[window][metric + "_percent_change_full_vs_smooth"] = (
                    100.0 * (old / new - 1.0) if old is not None and new is not None and new != 0.0 else None
                )
        pairs[full_row + "_vs_" + smooth_row] = pair
    result["evaluation_lock"] = {
        "path": str(lock_path), "sha256": lock_sha, "created_at_epoch_sec": lock.get("created_at_epoch_sec"),
        "primary_monitor": primary, "full_w_slosh": full_weight, "smooth_w_slosh": smooth_weight,
    }
    result["rows"] = ordered_reports
    result["pair_changes"] = pairs
    result["gate_failures"] = failures
    result["status"] = "PASS" if not failures else "FAIL"
    result["effect_claim_allowed"] = result["status"] == "PASS"
    result["pair_changes_role"] = "evaluation" if result["status"] == "PASS" else "diagnostic_only"
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--compare", type=Path, nargs="+")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--evaluation-lock", type=Path)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.compare:
            if args.output is None:
                raise ValueError("--output is required with --compare")
            result = compare_reports(args.compare, args.output, args.evaluation_lock)
            return 0 if result["status"] in ("PASS", "SCREENING") else 1
        if args.bag is None or args.report is None:
            raise ValueError("single mode requires --bag and --report")
        cache = args.report.with_name(args.report.stem + "_extracted.json")
        topics, cache_hit = load_cached_topics(args.bag, cache)
        report = analyze_topics(topics, args.bag)
        report["cache"] = {"path": str(cache), "hit": cache_hit, "topics": sorted(TOPICS.values())}
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 0 if report["status"] == "PASS" else 1
    except Exception as exc:
        if args.report is not None and not args.compare:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            bag = args.bag or Path("")
            prereg = bag.with_name(bag.stem + "_runtime_smoke_prereg.env") if args.bag else None
            args.report.write_text(json.dumps({
                "bag": str(args.bag), "status": "ERROR", "error": str(exc),
                "prereg_file": str(prereg) if prereg else None,
                "prereg": _read_env(prereg) if prereg else {},
            }, indent=2) + "\n")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
