#!/usr/bin/env python3
"""Independent Stage 2f/2g numerical reference.

This module deliberately does not import production code or the Stage 2e
complete-map reference.  ``known_prefix_cases`` exercises retrospective
history propagation; every ``nominal_commit_cases`` entry names one prefix
case and consumes its independently recomputed pose.  The JSON is the only
hand-written source of scenario values; expected values are checked by
recomputing them here.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

_INT64_MIN = -(1 << 63)
_INT64_MAX = (1 << 63) - 1
_UINT64_MAX = (1 << 64) - 1
_SIZE_T_MAX = _UINT64_MAX


class FixtureError(ValueError):
    """Raised when a Stage 2f/2g fixture violates its frozen schema."""


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise FixtureError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


def _reject_nonfinite_constant(value: str) -> None:
    raise FixtureError(f"non-finite JSON number is forbidden: {value}")


def _exact(value: Any, keys: Sequence[str], path: str) -> Mapping[str, Any]:
    if type(value) is not dict:
        raise FixtureError(f"{path} must be an object")
    actual = set(value)
    wanted = set(keys)
    if unknown := sorted(actual - wanted):
        raise FixtureError(f"{path} has unknown fields: {', '.join(unknown)}")
    if missing := sorted(wanted - actual):
        raise FixtureError(f"{path} is missing fields: {', '.join(missing)}")
    return value


def _string(value: Any, path: str) -> str:
    if type(value) is not str or not value:
        raise FixtureError(f"{path} must be a non-empty string")
    return value


def _finite(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FixtureError(f"{path} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise FixtureError(f"{path} must be a finite number")
    return converted


def _integer(value: Any, path: str, *, unsigned: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise FixtureError(f"{path} must be an integer")
    lower, upper = (0, _UINT64_MAX) if unsigned else (_INT64_MIN, _INT64_MAX)
    if value < lower or value > upper:
        raise FixtureError(f"{path} is outside its C++ integer range")
    return value


def _nonnegative_int64(value: Any, path: str) -> int:
    result = _integer(value, path)
    if result < 0:
        raise FixtureError(f"{path} must be nonnegative")
    return result


def _array(value: Any, path: str, *, minimum: int = 0) -> list[Any]:
    if type(value) is not list or len(value) < minimum:
        raise FixtureError(f"{path} must be an array with at least {minimum} items")
    return value


def _finite_array(value: Any, path: str) -> list[float]:
    return [
        _finite(item, f"{path}[{index}]")
        for index, item in enumerate(_array(value, path))
    ]


def _physical(value: Any, path: str) -> dict[str, Any]:
    obj = _exact(value, ("pose", "actual", "liquid"), path)
    pose = _exact(obj["pose"], ("x", "y", "heading"), f"{path}.pose")
    actual = _exact(
        obj["actual"], ("linear_velocity", "angular_velocity"), f"{path}.actual"
    )
    liquid = _exact(
        obj["liquid"], ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot"), f"{path}.liquid"
    )
    return {
        "pose": {
            key: _finite(pose[key], f"{path}.pose.{key}")
            for key in ("x", "y", "heading")
        },
        "actual": {
            key: _finite(actual[key], f"{path}.actual.{key}")
            for key in ("linear_velocity", "angular_velocity")
        },
        "liquid": {
            key: _finite(liquid[key], f"{path}.liquid.{key}")
            for key in ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot")
        },
    }


def _publisher(value: Any, path: str) -> dict[str, float]:
    keys = (
        "previous_linear_command",
        "previous_angular_command",
        "previous_linear_acceleration",
        "previous_angular_acceleration",
    )
    obj = _exact(value, keys, path)
    return {key: _finite(obj[key], f"{path}.{key}") for key in keys}


def _config(value: Any, path: str) -> dict[str, float]:
    keys = (
        "dt_sec",
        "maximum_linear_delay_sec",
        "maximum_angular_delay_sec",
        "linear_delay_sec",
        "angular_delay_sec",
        "integer_snap_tolerance_ratio",
        "duration_tolerance_sec",
    )
    obj = _exact(value, keys, path)
    config = {key: _finite(obj[key], f"{path}.{key}") for key in keys}
    if config["dt_sec"] <= 0.0:
        raise FixtureError(f"{path}.dt_sec must be positive")
    for key in ("maximum_linear_delay_sec", "maximum_angular_delay_sec"):
        if config[key] < 0.0:
            raise FixtureError(f"{path}.{key} must be nonnegative")
    for key in ("linear_delay_sec", "angular_delay_sec"):
        if config[key] < 0.0:
            raise FixtureError(f"{path}.{key} must be nonnegative")
    if config["linear_delay_sec"] > config["maximum_linear_delay_sec"]:
        raise FixtureError(f"{path}.linear_delay_sec exceeds maximum")
    if config["angular_delay_sec"] > config["maximum_angular_delay_sec"]:
        raise FixtureError(f"{path}.angular_delay_sec exceeds maximum")
    if not 0.0 <= config["integer_snap_tolerance_ratio"] < 0.5:
        raise FixtureError(f"{path}.integer_snap_tolerance_ratio is invalid")
    if not 0.0 <= config["duration_tolerance_sec"] < config["dt_sec"]:
        raise FixtureError(f"{path}.duration_tolerance_sec is invalid")
    return config


def _plant(value: Any, path: str) -> dict[str, Any]:
    obj = _exact(value, ("linear_actuator", "angular_actuator", "liquid"), path)
    channels: dict[str, dict[str, float]] = {}
    for name in ("linear_actuator", "angular_actuator"):
        channel = _exact(obj[name], ("tau_sec", "gain"), f"{path}.{name}")
        parsed = {
            key: _finite(channel[key], f"{path}.{name}.{key}")
            for key in ("tau_sec", "gain")
        }
        if parsed["tau_sec"] <= 0.0 or parsed["gain"] <= 0.0:
            raise FixtureError(f"{path}.{name} tau_sec and gain must be positive")
        channels[name] = parsed
    liquid_keys = (
        "natural_frequency_rad_per_sec",
        "damping_ratio",
        "longitudinal_coupling",
        "lateral_coupling",
    )
    liquid = _exact(obj["liquid"], liquid_keys, f"{path}.liquid")
    parsed_liquid = {
        key: _finite(liquid[key], f"{path}.liquid.{key}") for key in liquid_keys
    }
    if parsed_liquid["natural_frequency_rad_per_sec"] <= 0.0:
        raise FixtureError(f"{path}.liquid natural frequency must be positive")
    if parsed_liquid["damping_ratio"] < 0.0:
        raise FixtureError(f"{path}.liquid damping ratio must be nonnegative")
    for key in ("longitudinal_coupling", "lateral_coupling"):
        if parsed_liquid[key] <= 0.0:
            raise FixtureError(f"{path}.liquid.{key} must be positive")
    channels["liquid"] = parsed_liquid
    return channels


def _dimensions(maximum_delay: float, dt: float) -> dict[str, int]:
    count = math.ceil(maximum_delay / dt)
    return {"selector_width": count + 1, "older_count": max(0, count - 1)}


def _clock(value: Any, path: str) -> dict[str, int]:
    obj = _exact(value, ("anchor_steady_ns", "anchor_model_ns"), path)
    return {
        key: _integer(obj[key], f"{path}.{key}")
        for key in ("anchor_steady_ns", "anchor_model_ns")
    }


def _offset_ns(cycle_id: int) -> int:
    # ReleaseGridContract::boundaryOffsetNs: round-nearest(k * 1e9 / 30).
    if cycle_id < 0 or cycle_id > _UINT64_MAX:
        raise FixtureError("release cycle ID is outside uint64")
    whole, remainder = divmod(cycle_id, 30)
    offset = whole * 1_000_000_000 + (remainder * 1_000_000_000 + 15) // 30
    if offset > _INT64_MAX:
        raise FixtureError("release grid offset exceeds int64")
    return offset


def cycle_times(clock: Mapping[str, int], cycle_id: int) -> dict[str, int]:
    offset = _offset_ns(cycle_id)
    result = {
        "release_steady_ns": clock["anchor_steady_ns"] + offset,
        "release_model_ns": clock["anchor_model_ns"] + offset,
    }
    if any(value < _INT64_MIN or value > _INT64_MAX for value in result.values()):
        raise FixtureError("release boundary exceeds int64")
    return result


def _event(value: Any, path: str) -> dict[str, Any]:
    keys = (
        "cycle_id",
        "release_generation",
        "emission_reason",
        "linear_command",
        "angular_command",
        "linear_acceleration",
        "angular_acceleration",
        "actual_lateness_ns",
    )
    obj = _exact(value, keys, path)
    event = {
        "cycle_id": _integer(obj["cycle_id"], f"{path}.cycle_id", unsigned=True),
        "release_generation": _integer(
            obj["release_generation"], f"{path}.release_generation", unsigned=True
        ),
        "emission_reason": _string(obj["emission_reason"], f"{path}.emission_reason"),
        "linear_command": _finite(obj["linear_command"], f"{path}.linear_command"),
        "angular_command": _finite(obj["angular_command"], f"{path}.angular_command"),
        "linear_acceleration": _finite(
            obj["linear_acceleration"], f"{path}.linear_acceleration"
        ),
        "angular_acceleration": _finite(
            obj["angular_acceleration"], f"{path}.angular_acceleration"
        ),
        "actual_lateness_ns": _nonnegative_int64(
            obj["actual_lateness_ns"], f"{path}.actual_lateness_ns"
        ),
    }
    if event["release_generation"] == 0:
        raise FixtureError(f"{path}.release_generation must be positive")
    if event["emission_reason"] != "nominal":
        raise FixtureError(f"{path}.emission_reason must be nominal")
    return event


def _state(
    value: Any, path: str, linear_count: int, angular_count: int
) -> dict[str, Any]:
    obj = _exact(
        value, ("physical", "publisher", "linear_older", "angular_older"), path
    )
    linear = _finite_array(obj["linear_older"], f"{path}.linear_older")
    angular = _finite_array(obj["angular_older"], f"{path}.angular_older")
    if len(linear) != linear_count:
        raise FixtureError(f"{path}.linear_older must have length {linear_count}")
    if len(angular) != angular_count:
        raise FixtureError(f"{path}.angular_older must have length {angular_count}")
    return {
        "physical": _physical(obj["physical"], f"{path}.physical"),
        "publisher": _publisher(obj["publisher"], f"{path}.publisher"),
        "linear_older": linear,
        "angular_older": angular,
    }


def _segment(value: Any, path: str) -> dict[str, float]:
    keys = ("duration_sec", "linear_target", "angular_target")
    obj = _exact(value, keys, path)
    result = {key: _finite(obj[key], f"{path}.{key}") for key in keys}
    if result["duration_sec"] <= 0.0:
        raise FixtureError(f"{path}.duration_sec must be positive")
    return result


def _validate_known_case(value: Any, index: int) -> dict[str, Any]:
    path = f"fixture.known_prefix_cases[{index}]"
    keys = (
        "id",
        "history_capacity",
        "maximum_history_gap_ns",
        "maximum_publish_lateness_ns",
        "snapshot_lateness_ns",
        "reset_epoch",
        "clock",
        "config",
        "plant",
        "start_model_ns",
        "target_cycle_id",
        "initial_physical",
        "history",
        "expected",
    )
    obj = _exact(value, keys, path)
    result: dict[str, Any] = {"id": _string(obj["id"], f"{path}.id")}
    if result["id"] != "KnownPrefixNonuniformPhysicalGolden":
        raise FixtureError(f"{path}.id is not the frozen Stage 2f case")
    result["history_capacity"] = _integer(
        obj["history_capacity"], f"{path}.history_capacity", unsigned=True
    )
    if result["history_capacity"] < 2:
        raise FixtureError(f"{path}.history_capacity must be at least two")
    if result["history_capacity"] > _SIZE_T_MAX:
        raise FixtureError(f"{path}.history_capacity exceeds size_t")
    result["maximum_history_gap_ns"] = _nonnegative_int64(
        obj["maximum_history_gap_ns"], f"{path}.maximum_history_gap_ns"
    )
    result["maximum_publish_lateness_ns"] = _nonnegative_int64(
        obj["maximum_publish_lateness_ns"],
        f"{path}.maximum_publish_lateness_ns",
    )
    result["snapshot_lateness_ns"] = _nonnegative_int64(
        obj["snapshot_lateness_ns"], f"{path}.snapshot_lateness_ns"
    )
    result["reset_epoch"] = _integer(
        obj["reset_epoch"], f"{path}.reset_epoch", unsigned=True
    )
    result["clock"] = _clock(obj["clock"], f"{path}.clock")
    result["config"] = _config(obj["config"], f"{path}.config")
    frozen_period_sec = 1.0 / 30.0
    if (
        abs(result["config"]["dt_sec"] - frozen_period_sec)
        > result["config"]["duration_tolerance_sec"]
    ):
        raise FixtureError(f"{path}.config.dt_sec does not match the release grid")
    result["plant"] = _plant(obj["plant"], f"{path}.plant")
    result["start_model_ns"] = _integer(obj["start_model_ns"], f"{path}.start_model_ns")
    result["target_cycle_id"] = _integer(
        obj["target_cycle_id"], f"{path}.target_cycle_id", unsigned=True
    )
    result["initial_physical"] = _physical(
        obj["initial_physical"], f"{path}.initial_physical"
    )
    history = [
        _event(item, f"{path}.history[{i}]")
        for i, item in enumerate(_array(obj["history"], f"{path}.history", minimum=1))
    ]
    if len(history) > result["history_capacity"]:
        raise FixtureError(f"{path}.history exceeds history_capacity")
    if history[0]["release_generation"] != 1:
        raise FixtureError(f"{path}.history must start at release generation one")
    if result["target_cycle_id"] == 0:
        raise FixtureError(f"{path}.target_cycle_id must be positive")
    linear_dimensions = _dimensions(
        result["config"]["maximum_linear_delay_sec"], result["config"]["dt_sec"]
    )
    angular_dimensions = _dimensions(
        result["config"]["maximum_angular_delay_sec"], result["config"]["dt_sec"]
    )
    if (
        len(history)
        < max(linear_dimensions["older_count"], angular_dimensions["older_count"]) + 1
    ):
        raise FixtureError(f"{path}.history is too short to reconstruct older taps")
    for event_index, event in enumerate(history):
        if event_index == 0:
            continue
        previous = history[event_index - 1]
        if event["cycle_id"] != previous["cycle_id"] + 1:
            raise FixtureError(f"{path}.history cycle IDs must be contiguous")
        if event["release_generation"] != previous["release_generation"] + 1:
            raise FixtureError(f"{path}.history generations must be contiguous")
    if history[-1]["cycle_id"] + 1 != result["target_cycle_id"]:
        raise FixtureError(f"{path}.history latest cycle must be target_cycle_id - 1")
    release_times = [
        cycle_times(result["clock"], event["cycle_id"]) for event in history
    ]
    actual_steady = []
    actual_model = []
    for event, release in zip(history, release_times):
        if event["actual_lateness_ns"] > result["maximum_publish_lateness_ns"]:
            raise FixtureError(f"{path}.history actual lateness exceeds publish gate")
        event_actual_steady = release["release_steady_ns"] + event["actual_lateness_ns"]
        event_actual_model = release["release_model_ns"] + event["actual_lateness_ns"]
        if event_actual_steady > _INT64_MAX or event_actual_model > _INT64_MAX:
            raise FixtureError(f"{path}.history receipt time exceeds int64")
        actual_steady.append(event_actual_steady)
        actual_model.append(event_actual_model)
        if (
            actual_steady[-1] < release["release_steady_ns"]
            or actual_model[-1] < release["release_model_ns"]
        ):
            raise FixtureError(f"{path}.history receipt precedes release")
    for receipt_index in range(1, len(actual_steady)):
        if (
            actual_steady[receipt_index] <= actual_steady[receipt_index - 1]
            or actual_model[receipt_index] <= actual_model[receipt_index - 1]
        ):
            raise FixtureError(
                f"{path}.history receipt clocks must be strictly increasing"
            )
    latest_release = release_times[-1]
    target_release = cycle_times(result["clock"], result["target_cycle_id"])
    snapshot_steady = (
        latest_release["release_steady_ns"] + result["snapshot_lateness_ns"]
    )
    snapshot_model = latest_release["release_model_ns"] + result["snapshot_lateness_ns"]
    if snapshot_steady < actual_steady[-1] or snapshot_model < actual_model[-1]:
        raise FixtureError(f"{path}.snapshot must not precede latest receipt")
    if (
        not result["start_model_ns"]
        <= snapshot_model
        <= target_release["release_model_ns"]
    ):
        raise FixtureError(f"{path}.start/snapshot/target model range is invalid")
    if snapshot_steady > target_release["release_steady_ns"]:
        raise FixtureError(f"{path}.snapshot steady time exceeds target")
    result["history"] = history
    expected_obj = _exact(
        obj["expected"],
        (
            "status",
            "state",
            "segments",
            "segment_count",
            "history_generation",
            "last_emitted_cycle_id",
            "start_model_ns",
            "target_model_ns",
            "coverage",
        ),
        f"{path}.expected",
    )
    result["expected"] = _validate_known_expected(
        expected_obj, f"{path}.expected", result
    )
    return result


def _coverage(value: Any, path: str) -> dict[str, Any]:
    keys = (
        "status",
        "history_generation",
        "predecessor_release_generation",
        "covered_event_count",
        "maximum_adjacent_gap_ns",
        "future_hold_ns",
        "maximum_required_gap_ns",
    )
    obj = _exact(value, keys, path)
    result: dict[str, Any] = {"status": _string(obj["status"], f"{path}.status")}
    for key in keys[1:]:
        result[key] = _integer(obj[key], f"{path}.{key}", unsigned=True)
    return result


def _validate_known_expected(
    value: Mapping[str, Any], path: str, case: Mapping[str, Any]
) -> dict[str, Any]:
    config = case["config"]
    dimensions_linear = _dimensions(
        config["maximum_linear_delay_sec"], config["dt_sec"]
    )
    dimensions_angular = _dimensions(
        config["maximum_angular_delay_sec"], config["dt_sec"]
    )
    result: dict[str, Any] = {
        "status": _string(value["status"], f"{path}.status"),
        "state": _state(
            value["state"],
            f"{path}.state",
            dimensions_linear["older_count"],
            dimensions_angular["older_count"],
        ),
        "segments": [
            _segment(item, f"{path}.segments[{i}]")
            for i, item in enumerate(
                _array(value["segments"], f"{path}.segments", minimum=1)
            )
        ],
        "segment_count": _integer(
            value["segment_count"], f"{path}.segment_count", unsigned=True
        ),
        "history_generation": _integer(
            value["history_generation"], f"{path}.history_generation", unsigned=True
        ),
        "last_emitted_cycle_id": _integer(
            value["last_emitted_cycle_id"],
            f"{path}.last_emitted_cycle_id",
            unsigned=True,
        ),
        "start_model_ns": _integer(value["start_model_ns"], f"{path}.start_model_ns"),
        "target_model_ns": _integer(
            value["target_model_ns"], f"{path}.target_model_ns"
        ),
        "coverage": _coverage(value["coverage"], f"{path}.coverage"),
    }
    if result["segment_count"] != len(result["segments"]):
        raise FixtureError(f"{path}.segment_count does not match segments")
    return result


def _identity(value: Any, path: str) -> dict[str, Any]:
    obj = _exact(value, ("path_id", "path_hash", "reset_epoch"), path)
    path_id = _integer(obj["path_id"], f"{path}.path_id", unsigned=True)
    path_hash = _string(obj["path_hash"], f"{path}.path_hash").lower()
    if len(path_hash) != 64 or any(
        char not in "0123456789abcdef" for char in path_hash
    ):
        raise FixtureError(f"{path}.path_hash must be 64 hexadecimal characters")
    if set(path_hash) == {"0"}:
        raise FixtureError(f"{path}.path_hash must be nonzero")
    return {
        "path_id": path_id,
        "path_hash": path_hash,
        "reset_epoch": _integer(
            obj["reset_epoch"], f"{path}.reset_epoch", unsigned=True
        ),
    }


def _path(value: Any, path: str) -> dict[str, Any]:
    obj = _exact(value, ("identity", "vertices", "s_path_end"), path)
    vertices = []
    for index, item in enumerate(
        _array(obj["vertices"], f"{path}.vertices", minimum=2)
    ):
        vertex = _exact(item, ("x", "y", "cumulative_s"), f"{path}.vertices[{index}]")
        vertices.append(
            {
                key: _finite(vertex[key], f"{path}.vertices[{index}].{key}")
                for key in ("x", "y", "cumulative_s")
            }
        )
    if len(vertices) != 2:
        raise FixtureError(f"{path}.vertices must contain the frozen two-point path")
    return {
        "identity": _identity(obj["identity"], f"{path}.identity"),
        "vertices": vertices,
        "s_path_end": _finite(obj["s_path_end"], f"{path}.s_path_end"),
    }


def _validate_projection_expected(value: Any, path: str) -> dict[str, Any]:
    keys = (
        "status",
        "s",
        "projected_x",
        "projected_y",
        "projected_heading",
        "distance",
        "signed_contour_error",
        "heading_error",
        "search_lower",
        "search_upper",
        "selected_segment",
        "accepted_candidate_count",
        "authority_cycle_id",
        "history_generation",
    )
    obj = _exact(value, keys, path)
    result: dict[str, Any] = {"status": _string(obj["status"], f"{path}.status")}
    for key in keys[1:10]:
        result[key] = _finite(obj[key], f"{path}.{key}")
    for key in (
        "selected_segment",
        "accepted_candidate_count",
        "authority_cycle_id",
        "history_generation",
    ):
        result[key] = _integer(obj[key], f"{path}.{key}", unsigned=True)
    return result


def _validate_projection_case(
    value: Any, index: int, known_ids: set[str]
) -> dict[str, Any]:
    path = f"fixture.nominal_commit_cases[{index}]"
    common = (
        "id",
        "source_known_prefix_case_id",
        "config",
        "path",
        "authority",
        "expected",
    )
    obj = _exact(value, common, path)
    result: dict[str, Any] = {"id": _string(obj["id"], f"{path}.id")}
    if result["id"] != "NominalStraightPoseProjectionGolden":
        raise FixtureError(f"{path}.id is not the frozen Stage 2g case")
    result["source_known_prefix_case_id"] = _string(
        obj["source_known_prefix_case_id"],
        f"{path}.source_known_prefix_case_id",
    )
    if result["source_known_prefix_case_id"] not in known_ids:
        raise FixtureError(f"{path}.source_known_prefix_case_id is unknown")
    config_keys = (
        "start_s_min",
        "start_s_max",
        "v_progress_bound",
        "forward_guard",
        "contour_guard",
        "heading_guard",
        "ambiguity_tolerance",
        "progress_equivalence_tolerance",
        "minimum_segment_length",
    )
    config_obj = _exact(obj["config"], config_keys, f"{path}.config")
    result["config"] = {
        key: _finite(config_obj[key], f"{path}.config.{key}") for key in config_keys
    }
    if (
        result["config"]["start_s_min"] < 0.0
        or result["config"]["start_s_max"] < result["config"]["start_s_min"]
        or result["config"]["v_progress_bound"] < 0.0
        or result["config"]["forward_guard"] < 0.0
        or result["config"]["contour_guard"] < 0.0
        or result["config"]["heading_guard"] < 0.0
        or result["config"]["heading_guard"] > math.pi
        or result["config"]["ambiguity_tolerance"] < 0.0
        or result["config"]["progress_equivalence_tolerance"] < 0.0
        or result["config"]["minimum_segment_length"] <= 0.0
        or result["config"]["progress_equivalence_tolerance"]
        >= result["config"]["minimum_segment_length"]
    ):
        raise FixtureError(f"{path}.config violates production projector contract")
    result["path"] = _path(obj["path"], f"{path}.path")
    authority_keys = (
        "kind",
        "identity",
        "s_commit",
        "release_cycle_id",
        "release_generation",
        "history_generation",
    )
    authority_obj = _exact(obj["authority"], authority_keys, f"{path}.authority")
    authority = {
        "kind": _string(authority_obj["kind"], f"{path}.authority.kind"),
        "identity": _identity(authority_obj["identity"], f"{path}.authority.identity"),
        "s_commit": _finite(authority_obj["s_commit"], f"{path}.authority.s_commit"),
        "release_cycle_id": _integer(
            authority_obj["release_cycle_id"],
            f"{path}.authority.release_cycle_id",
            unsigned=True,
        ),
        "release_generation": _integer(
            authority_obj["release_generation"],
            f"{path}.authority.release_generation",
            unsigned=True,
        ),
        "history_generation": _integer(
            authority_obj["history_generation"],
            f"{path}.authority.history_generation",
            unsigned=True,
        ),
    }
    if authority["kind"] != "nominal_live_release":
        raise FixtureError(f"{path}.authority.kind must be nominal_live_release")
    result["authority"] = authority
    result["expected"] = _validate_projection_expected(
        obj["expected"], f"{path}.expected"
    )
    return result


def load_fixture(path: Path | str) -> dict[str, Any]:
    fixture_path = Path(path)
    try:
        text = fixture_path.read_text(encoding="utf-8")
        value = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite_constant,
        )
    except FixtureError:
        raise
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise FixtureError(f"cannot load fixture {fixture_path}: {exc}") from exc
    root = _exact(
        value,
        ("schema_version", "numeric", "known_prefix_cases", "nominal_commit_cases"),
        "fixture",
    )
    if root["schema_version"] != "stage2_fg_execution_projection_golden_v1":
        raise FixtureError(
            "fixture.schema_version is not stage2_fg_execution_projection_golden_v1"
        )
    numeric = _exact(root["numeric"], ("absolute_tolerance",), "fixture.numeric")
    tolerance = _finite(
        numeric["absolute_tolerance"], "fixture.numeric.absolute_tolerance"
    )
    if tolerance <= 0.0:
        raise FixtureError("fixture.numeric.absolute_tolerance must be positive")
    known = [
        _validate_known_case(item, i)
        for i, item in enumerate(
            _array(root["known_prefix_cases"], "fixture.known_prefix_cases", minimum=1)
        )
    ]
    if len(known) != 1:
        raise FixtureError("fixture.known_prefix_cases must contain exactly one case")
    known_ids = {case["id"] for case in known}
    nominal = [
        _validate_projection_case(item, i, known_ids)
        for i, item in enumerate(
            _array(
                root["nominal_commit_cases"], "fixture.nominal_commit_cases", minimum=1
            )
        )
    ]
    if len(nominal) != 1:
        raise FixtureError("fixture.nominal_commit_cases must contain exactly one case")
    if nominal[0]["id"] != "NominalStraightPoseProjectionGolden":
        raise FixtureError("fixture.nominal_commit_cases has an unexpected fixed ID")
    return {
        "schema_version": root["schema_version"],
        "numeric": {"absolute_tolerance": tolerance},
        "known_prefix_cases": known,
        "nominal_commit_cases": nominal,
    }


def _fopdt(
    actual: float, target: float, duration: float, channel: Mapping[str, float]
) -> float:
    exponent = duration / channel["tau_sec"]
    rho = math.exp(-exponent)
    return rho * actual + (-math.expm1(-exponent)) * channel["gain"] * target


def _actual_at(
    initial: Mapping[str, float],
    segment: Mapping[str, float],
    plant: Mapping[str, Any],
    elapsed: float,
) -> dict[str, float]:
    return {
        "linear_velocity": _fopdt(
            initial["linear_velocity"],
            segment["linear_target"],
            elapsed,
            plant["linear_actuator"],
        ),
        "angular_velocity": _fopdt(
            initial["angular_velocity"],
            segment["angular_target"],
            elapsed,
            plant["angular_actuator"],
        ),
    }


def _liquid_derivative(
    liquid: Mapping[str, float],
    initial_actual: Mapping[str, float],
    segment: Mapping[str, float],
    plant: Mapping[str, Any],
    elapsed: float,
) -> list[float]:
    actual = _actual_at(initial_actual, segment, plant, elapsed)
    params = plant["liquid"]
    damping = 2.0 * params["damping_ratio"] * params["natural_frequency_rad_per_sec"]
    stiffness = params["natural_frequency_rad_per_sec"] ** 2
    longitudinal = (
        plant["linear_actuator"]["gain"] * segment["linear_target"]
        - actual["linear_velocity"]
    ) / plant["linear_actuator"]["tau_sec"]
    lateral = actual["linear_velocity"] * actual["angular_velocity"]
    return [
        liquid["eta_x_dot"],
        -damping * liquid["eta_x_dot"]
        - stiffness * liquid["eta_x"]
        - params["longitudinal_coupling"] * longitudinal,
        liquid["eta_y_dot"],
        -damping * liquid["eta_y_dot"]
        - stiffness * liquid["eta_y"]
        - params["lateral_coupling"] * lateral,
    ]


def _propagate(
    physical: Mapping[str, Any], segment: Mapping[str, float], plant: Mapping[str, Any]
) -> dict[str, Any]:
    duration = segment["duration_sec"]
    if duration == 0.0:
        return {key: dict(value) for key, value in physical.items()}
    midpoint = _actual_at(physical["actual"], segment, plant, 0.5 * duration)
    end = _actual_at(physical["actual"], segment, plant, duration)
    half = 0.5 * duration
    k1 = _liquid_derivative(physical["liquid"], physical["actual"], segment, plant, 0.0)
    k2_state = {
        key: physical["liquid"][key] + half * k1[index]
        for index, key in enumerate(("eta_x", "eta_x_dot", "eta_y", "eta_y_dot"))
    }
    k2 = _liquid_derivative(k2_state, physical["actual"], segment, plant, half)
    k3_state = {
        key: physical["liquid"][key] + half * k2[index]
        for index, key in enumerate(("eta_x", "eta_x_dot", "eta_y", "eta_y_dot"))
    }
    k3 = _liquid_derivative(k3_state, physical["actual"], segment, plant, half)
    k4_state = {
        key: physical["liquid"][key] + duration * k3[index]
        for index, key in enumerate(("eta_x", "eta_x_dot", "eta_y", "eta_y_dot"))
    }
    k4 = _liquid_derivative(k4_state, physical["actual"], segment, plant, duration)
    keys = ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot")
    liquid = {
        key: physical["liquid"][key]
        + duration / 6.0 * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index])
        for index, key in enumerate(keys)
    }
    heading_delta = duration * midpoint["angular_velocity"]
    heading_midpoint = physical["pose"]["heading"] + 0.5 * heading_delta
    pose = {
        "x": physical["pose"]["x"]
        + duration * midpoint["linear_velocity"] * math.cos(heading_midpoint),
        "y": physical["pose"]["y"]
        + duration * midpoint["linear_velocity"] * math.sin(heading_midpoint),
        "heading": physical["pose"]["heading"] + heading_delta,
    }
    return {"pose": pose, "actual": end, "liquid": liquid}


def _schedule(delay: float, config: Mapping[str, float]) -> dict[str, Any]:
    dt = config["dt_sec"]
    ratio = delay / dt
    lower = math.floor(ratio)
    beta = ratio - lower
    snap = config["integer_snap_tolerance_ratio"]
    if beta <= snap:
        beta = 0.0
    elif 1.0 - beta <= snap:
        beta = 0.0
        lower += 1
    return {"integer": lower, "fraction": beta, "switch": beta * dt}


def _close_time(left: float, right: float) -> bool:
    scale = max(1.0, abs(left), abs(right))
    return abs(left - right) <= 8.0 * 2.220446049250313e-16 * scale


def _maximum_delay_ns(config: Mapping[str, float]) -> int:
    maximum_delay = max(
        config["maximum_linear_delay_sec"],
        config["maximum_angular_delay_sec"],
    )
    nanoseconds = maximum_delay * 1e9
    nearest = round(nanoseconds)
    tolerance_ns = config["duration_tolerance_sec"] * 1e9
    normalized = nearest if abs(nanoseconds - nearest) <= tolerance_ns else nanoseconds
    result = math.ceil(normalized)
    if result < 0 or result > _INT64_MAX:
        raise FixtureError("known-prefix maximum delay exceeds int64")
    return result


def calculate_known_prefix(case: Mapping[str, Any]) -> dict[str, Any]:
    if case["id"] != "KnownPrefixNonuniformPhysicalGolden":
        raise FixtureError("unknown Stage 2f case ID")
    config = case["config"]
    linear_dim = _dimensions(config["maximum_linear_delay_sec"], config["dt_sec"])
    angular_dim = _dimensions(config["maximum_angular_delay_sec"], config["dt_sec"])
    history = case["history"]
    target_times = cycle_times(case["clock"], case["target_cycle_id"])
    events = []
    for event in history:
        times = cycle_times(case["clock"], event["cycle_id"])
        events.append(
            {
                **event,
                **times,
                "actual_steady_ns": times["release_steady_ns"]
                + event["actual_lateness_ns"],
                "actual_model_ns": times["release_model_ns"]
                + event["actual_lateness_ns"],
            }
        )
    if not events:
        raise FixtureError("known-prefix history must not be empty")
    latest = events[-1]
    for event_index, event in enumerate(events):
        if event["emission_reason"] != "nominal":
            raise FixtureError("known-prefix history contains an unsupported reason")
        if event["actual_lateness_ns"] > case["maximum_publish_lateness_ns"]:
            raise FixtureError("known-prefix receipt exceeds publish lateness gate")
        if event_index > 0:
            previous = events[event_index - 1]
            if event["cycle_id"] != previous["cycle_id"] + 1:
                raise FixtureError("known-prefix cycle IDs are not contiguous")
            if event["release_generation"] != previous["release_generation"] + 1:
                raise FixtureError("known-prefix generations are not contiguous")
            if (
                event["actual_steady_ns"] <= previous["actual_steady_ns"]
                or event["actual_model_ns"] <= previous["actual_model_ns"]
            ):
                raise FixtureError("known-prefix receipt clocks are not monotonic")
        if (
            event["actual_steady_ns"] < event["release_steady_ns"]
            or event["actual_model_ns"] < event["release_model_ns"]
        ):
            raise FixtureError("known-prefix receipt precedes release")
    if latest["cycle_id"] + 1 != case["target_cycle_id"]:
        raise FixtureError("known-prefix latest event is not target cycle - 1")
    start_model = case["start_model_ns"]
    snapshot_model = latest["release_model_ns"] + case["snapshot_lateness_ns"]
    snapshot_steady = latest["release_steady_ns"] + case["snapshot_lateness_ns"]
    if (
        snapshot_model < latest["actual_model_ns"]
        or snapshot_steady < latest["actual_steady_ns"]
    ):
        raise FixtureError("known-prefix snapshot precedes latest receipt")
    if not start_model <= snapshot_model <= target_times["release_model_ns"]:
        raise FixtureError("known-prefix start/snapshot/target model range is invalid")
    if snapshot_steady > target_times["release_steady_ns"]:
        raise FixtureError("known-prefix snapshot exceeds target steady time")
    total_duration = (target_times["release_model_ns"] - start_model) * 1e-9
    if total_duration < 0.0:
        raise FixtureError("known-prefix target precedes start")
    linear_timing = _schedule(config["linear_delay_sec"], config)
    angular_timing = _schedule(config["angular_delay_sec"], config)

    def effective(event: Mapping[str, Any], timing: Mapping[str, Any]) -> float:
        integer_delay_ns = _offset_ns(
            event["cycle_id"] + timing["integer"]
        ) - _offset_ns(event["cycle_id"])
        return (
            (event["release_model_ns"] - start_model) * 1e-9
            + integer_delay_ns * 1e-9
            + timing["switch"]
        )

    def channel_at(
        elapsed: float, timing: Mapping[str, Any], linear: bool
    ) -> tuple[float, float]:
        target = None
        next_switch = total_duration
        for event in events:
            effective_time = effective(event, timing)
            if _close_time(effective_time, elapsed):
                effective_time = elapsed
            elif _close_time(effective_time, total_duration):
                effective_time = total_duration
            value = event["linear_command"] if linear else event["angular_command"]
            if effective_time <= elapsed:
                target = value
            elif effective_time < total_duration:
                next_switch = min(next_switch, effective_time)
        if target is None:
            raise FixtureError("known-prefix fixture has no historical predecessor")
        return target, next_switch

    current = case["initial_physical"]
    segments: list[dict[str, float]] = []
    elapsed = 0.0
    while elapsed < total_duration:
        linear_target, linear_switch = channel_at(elapsed, linear_timing, True)
        angular_target, angular_switch = channel_at(elapsed, angular_timing, False)
        switch = min(linear_switch, angular_switch)
        segment = {
            "duration_sec": switch - elapsed,
            "linear_target": linear_target,
            "angular_target": angular_target,
        }
        if segment["duration_sec"] <= 0.0:
            raise FixtureError("known-prefix schedule contains an empty segment")
        segments.append(segment)
        if len(segments) > 2 * case["history_capacity"] + 1:
            raise FixtureError("known-prefix segment count exceeds result capacity")
        current = _propagate(current, segment, case["plant"])
        elapsed = switch

    def older(name: str, count: int) -> list[float]:
        return [event[name] for event in reversed(events[:-1])][:count]

    left_ns = start_model - _maximum_delay_ns(config)
    predecessor_index = None
    for index, event in enumerate(events):
        if event["release_model_ns"] <= left_ns:
            predecessor_index = index
    if predecessor_index is None:
        raise FixtureError("known-prefix fixture has no coverage predecessor")
    predecessor = events[predecessor_index]
    covered_events = events[predecessor_index:]
    gaps = [
        covered_events[index]["release_model_ns"]
        - covered_events[index - 1]["release_model_ns"]
        for index in range(1, len(covered_events))
    ]
    adjacent = max(gaps) if gaps else 0
    future = target_times["release_model_ns"] - latest["release_model_ns"]
    coverage = {
        "status": "complete",
        "history_generation": latest["release_generation"],
        "predecessor_release_generation": predecessor["release_generation"],
        "covered_event_count": len(covered_events),
        "maximum_adjacent_gap_ns": adjacent,
        "future_hold_ns": future,
        "maximum_required_gap_ns": max(adjacent, future),
    }
    if coverage["maximum_required_gap_ns"] > case["maximum_history_gap_ns"]:
        raise FixtureError("known-prefix history coverage gap exceeds configured gate")
    return {
        "status": "ok",
        "state": {
            "physical": current,
            "publisher": {
                "previous_linear_command": latest["linear_command"],
                "previous_angular_command": latest["angular_command"],
                "previous_linear_acceleration": latest["linear_acceleration"],
                "previous_angular_acceleration": latest["angular_acceleration"],
            },
            "linear_older": older("linear_command", linear_dim["older_count"]),
            "angular_older": older("angular_command", angular_dim["older_count"]),
        },
        "segments": segments,
        "segment_count": len(segments),
        "history_generation": latest["release_generation"],
        "last_emitted_cycle_id": latest["cycle_id"],
        "start_model_ns": start_model,
        "target_model_ns": target_times["release_model_ns"],
        "coverage": coverage,
    }


def _validate_path(case: Mapping[str, Any]) -> tuple[float, float, list[int]]:
    path = case["path"]
    vertices = path["vertices"]
    config = case["config"]
    minimum = config["minimum_segment_length"]
    equivalence = config["progress_equivalence_tolerance"]
    first = None
    last = None
    if vertices[0]["cumulative_s"] < 0.0:
        raise FixtureError("path cumulative progress must be nonnegative")
    for index in range(1, len(vertices)):
        a, b = vertices[index - 1], vertices[index]
        if b["cumulative_s"] < a["cumulative_s"]:
            raise FixtureError("path cumulative progress is not monotonic")
        length = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        progress_length = b["cumulative_s"] - a["cumulative_s"]
        if length < minimum:
            if progress_length > equivalence:
                raise FixtureError("zero-length path segment advances progress")
            continue
        scale = max(1.0, length, progress_length)
        tolerance = max(equivalence, 64.0 * 2.220446049250313e-16 * scale)
        if progress_length <= 0.0 or abs(length - progress_length) > tolerance:
            raise FixtureError("path arc length does not match geometry")
        if first is None:
            first = index - 1
        last = index - 1
    if (
        first is None
        or path["s_path_end"] != vertices[-1]["cumulative_s"]
        or path["s_path_end"] < vertices[0]["cumulative_s"]
    ):
        raise FixtureError("path has no usable segment or invalid endpoint")
    usable = [
        index
        for index in range(first, last + 1)
        if math.hypot(
            vertices[index + 1]["x"] - vertices[index]["x"],
            vertices[index + 1]["y"] - vertices[index]["y"],
        )
        >= minimum
    ]
    return vertices[0]["cumulative_s"], path["s_path_end"], usable


def _projection_window(
    case: Mapping[str, Any],
    pose: Mapping[str, float],
    lower: float,
    upper: float,
    authority_cycle_id: int,
) -> dict[str, Any]:
    path = case["path"]
    config = case["config"]
    path_start, path_end, usable = _validate_path(case)
    if lower < path_start or upper > path_end or upper < lower:
        raise FixtureError("projection window is outside path")
    candidates = []
    for index in usable:
        a, b = path["vertices"][index], path["vertices"][index + 1]
        if b["cumulative_s"] < lower or a["cumulative_s"] > upper:
            continue
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        length = math.hypot(dx, dy)
        tx, ty = dx / length, dy / length
        rx, ry = pose["x"] - a["x"], pose["y"] - a["y"]
        raw_t = (rx * tx + ry * ty) / length
        progress_length = b["cumulative_s"] - a["cumulative_s"]
        raw_s = a["cumulative_s"] + raw_t * progress_length
        true_start = index == usable[0] and lower == path_start and raw_t < 0.0
        true_end = index == usable[-1] and upper == path_end and raw_t > 1.0
        scale = max(1.0, abs(raw_s), abs(lower), abs(upper))
        boundary_tolerance = max(
            config["progress_equivalence_tolerance"],
            64.0 * 2.220446049250313e-16 * scale,
        )
        if raw_s < lower and lower - raw_s > boundary_tolerance and not true_start:
            continue
        if raw_s > upper and raw_s - upper > boundary_tolerance and not true_end:
            continue
        t = max(0.0, min(1.0, raw_t))
        s = a["cumulative_s"] + t * progress_length
        if not true_start and s < lower and lower - s <= boundary_tolerance:
            s = lower
            t = (lower - a["cumulative_s"]) / progress_length
        elif not true_end and s > upper and s - upper <= boundary_tolerance:
            s = upper
            t = (upper - a["cumulative_s"]) / progress_length
        x, y = a["x"] + t * dx, a["y"] + t * dy
        heading = math.atan2(dy, dx)
        error_x, error_y = pose["x"] - x, pose["y"] - y
        distance = math.hypot(error_x, error_y)
        contour = error_x * (-ty) + error_y * tx
        heading_error = math.atan2(
            math.sin(pose["heading"] - heading), math.cos(pose["heading"] - heading)
        )
        guarded_contour = distance if raw_t < 0.0 or raw_t > 1.0 else abs(contour)
        if (
            guarded_contour > config["contour_guard"]
            or abs(heading_error) > config["heading_guard"]
        ):
            continue
        candidates.append(
            {
                "s": s,
                "projected_x": x,
                "projected_y": y,
                "projected_heading": heading,
                "distance": distance,
                "signed_contour_error": contour,
                "heading_error": heading_error,
                "selected_segment": index,
            }
        )
    if not candidates:
        raise FixtureError("projection fixture does not contain an accepted candidate")
    candidates.sort(
        key=lambda item: (
            item["distance"],
            abs(item["heading_error"]),
            item["selected_segment"],
        )
    )
    selected = candidates[0]
    return {
        "status": "ok",
        **selected,
        "search_lower": lower,
        "search_upper": upper,
        "accepted_candidate_count": len(candidates),
        "authority_cycle_id": authority_cycle_id,
        "history_generation": case["authority"]["history_generation"],
    }


def _validate_projection_link(
    case: Mapping[str, Any],
    prefix_case: Mapping[str, Any],
    prefix_result: Mapping[str, Any],
) -> None:
    path = f"{case['id']}.authority"
    authority = case["authority"]
    prefix_identity = case["path"]["identity"]
    if authority["kind"] != "nominal_live_release":
        raise FixtureError(f"{path}.kind must be nominal_live_release")
    if prefix_case["reset_epoch"] != prefix_identity["reset_epoch"]:
        raise FixtureError(f"{case['id']}.path reset epoch does not match prefix")
    if authority["identity"] != prefix_identity:
        raise FixtureError(f"{path}.identity does not match path identity")
    if authority["identity"]["reset_epoch"] != prefix_case["reset_epoch"]:
        raise FixtureError(f"{path}.identity reset epoch does not match prefix")
    if authority["release_cycle_id"] + 1 != prefix_case["target_cycle_id"]:
        raise FixtureError(f"{path}.release_cycle_id must be target_cycle_id - 1")
    history_generation = prefix_result["history_generation"]
    if history_generation == 0:
        raise FixtureError(f"{case['id']} prefix history generation must be nonzero")
    if (
        authority["release_generation"] != history_generation
        or authority["history_generation"] != history_generation
    ):
        raise FixtureError(f"{path} generation does not match prefix result")
    path_start, path_end, _ = _validate_path(case)
    if authority["s_commit"] < path_start or authority["s_commit"] > path_end:
        raise FixtureError(f"{path}.s_commit is outside the path")


def calculate_nominal_commit(
    case: Mapping[str, Any],
    prefix_case: Mapping[str, Any],
    prefix_result: Mapping[str, Any],
) -> dict[str, Any]:
    _validate_projection_link(case, prefix_case, prefix_result)
    path_start, path_end, _ = _validate_path(case)
    authority = case["authority"]
    if authority["s_commit"] < path_start or authority["s_commit"] > path_end:
        raise FixtureError("nominal commit is outside path")
    target = cycle_times(prefix_case["clock"], prefix_case["target_cycle_id"])
    release = cycle_times(prefix_case["clock"], authority["release_cycle_id"])
    duration_sec = (target["release_model_ns"] - release["release_model_ns"]) * 1e-9
    upper = min(
        path_end,
        authority["s_commit"]
        + case["config"]["v_progress_bound"] * duration_sec
        + case["config"]["forward_guard"],
    )
    return _projection_window(
        case,
        prefix_result["state"]["physical"]["pose"],
        authority["s_commit"],
        upper,
        authority["release_cycle_id"],
    )


def _compare(
    expected: Any, actual: Any, tolerance: float, path: str = "expected"
) -> None:
    if isinstance(expected, bool) or isinstance(actual, bool):
        if expected != actual:
            raise FixtureError(f"{path} differs: expected {expected!r}, got {actual!r}")
        return
    if isinstance(expected, (int, str)) or isinstance(actual, (int, str)):
        if expected != actual:
            raise FixtureError(f"{path} differs: expected {expected!r}, got {actual!r}")
        return
    if isinstance(expected, float) or isinstance(actual, float):
        if not math.isclose(
            float(expected), float(actual), rel_tol=0.0, abs_tol=tolerance
        ):
            raise FixtureError(f"{path} differs: expected {expected!r}, got {actual!r}")
        return
    if isinstance(expected, Mapping) and isinstance(actual, Mapping):
        if set(expected) != set(actual):
            raise FixtureError(f"{path} keys differ")
        for key in expected:
            _compare(expected[key], actual[key], tolerance, f"{path}.{key}")
        return
    if isinstance(expected, Sequence) and isinstance(actual, Sequence):
        if len(expected) != len(actual):
            raise FixtureError(f"{path} lengths differ")
        for index, (left, right) in enumerate(zip(expected, actual)):
            _compare(left, right, tolerance, f"{path}[{index}]")
        return
    if expected != actual:
        raise FixtureError(f"{path} differs")


def validate_expected(fixture: Mapping[str, Any]) -> None:
    tolerance = fixture["numeric"]["absolute_tolerance"]
    known_cases: dict[str, Mapping[str, Any]] = {}
    known_results: dict[str, Mapping[str, Any]] = {}
    for case in fixture["known_prefix_cases"]:
        if case["id"] in known_results:
            raise FixtureError("known-prefix case IDs must be unique")
        calculated = calculate_known_prefix(case)
        _compare(
            case["expected"],
            calculated,
            tolerance,
            f"{case['id']}.expected",
        )
        known_cases[case["id"]] = case
        known_results[case["id"]] = calculated
    for case in fixture["nominal_commit_cases"]:
        source_id = case["source_known_prefix_case_id"]
        if source_id not in known_results:
            raise FixtureError(f"{case['id']} source prefix case is missing")
        calculated = calculate_nominal_commit(
            case,
            known_cases[source_id],
            known_results[source_id],
        )
        _compare(
            case["expected"],
            calculated,
            tolerance,
            f"{case['id']}.expected",
        )


def canonical_json(value: Mapping[str, Any]) -> str:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    )


def canonical_sha256(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()
