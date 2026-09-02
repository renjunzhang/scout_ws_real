#!/usr/bin/env python3
"""Independent Stage 2 execution golden reference.

The implementation in this module intentionally does not import production
code.  It is a small, auditable numerical reference for the first Stage 2e
fixture and is also the single parser used by the header generator and the
Python contract tests.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any


class FixtureError(ValueError):
    """Raised when a fixture is not an exact, finite Stage 2e document."""


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise FixtureError(f"duplicate JSON object key: {key!r}")
        result[key] = value
    return result


def _reject_nonfinite_constant(value: str) -> None:
    raise FixtureError(f"non-finite JSON number is forbidden: {value}")


def _exact_keys(value: Any, expected: Sequence[str], path: str) -> Mapping[str, Any]:
    if type(value) is not dict:
        raise FixtureError(f"{path} must be an object")
    actual = set(value)
    wanted = set(expected)
    unknown = sorted(actual - wanted)
    missing = sorted(wanted - actual)
    if unknown:
        raise FixtureError(f"{path} has unknown fields: {', '.join(unknown)}")
    if missing:
        raise FixtureError(f"{path} is missing fields: {', '.join(missing)}")
    return value


def _string(value: Any, path: str) -> str:
    if type(value) is not str or not value:
        raise FixtureError(f"{path} must be a non-empty string")
    return value


def _finite_number(value: Any, path: str) -> float:
    # bool is an int subclass, but it is not a JSON numeric field here.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FixtureError(f"{path} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise FixtureError(f"{path} must be a finite number")
    return converted


def delay_dimensions(maximum_delay_sec: float, dt_sec: float) -> dict[str, int]:
    """Return the frozen queue dimensions used by both fixture consumers."""
    if (
        not math.isfinite(maximum_delay_sec)
        or maximum_delay_sec < 0.0
        or not math.isfinite(dt_sec)
        or dt_sec <= 0.0
    ):
        raise FixtureError("delay dimensions require finite nonnegative L_max and dt")
    retained_command_count = math.ceil(maximum_delay_sec / dt_sec)
    return {
        "retained_command_count": retained_command_count,
        "older_count": max(0, retained_command_count - 1),
        "selector_width": retained_command_count + 1,
    }


def _array(value: Any, length: int, path: str) -> list[Any]:
    if type(value) is not list or len(value) != length:
        raise FixtureError(f"{path} must be an array of length {length}")
    return value


def _numeric_array(value: Any, length: int, path: str) -> list[float]:
    return [
        _finite_number(item, f"{path}[{index}]")
        for index, item in enumerate(_array(value, length, path))
    ]


def _pose(value: Any, path: str) -> dict[str, float]:
    obj = _exact_keys(value, ("x", "y", "heading"), path)
    return {
        key: _finite_number(obj[key], f"{path}.{key}") for key in ("x", "y", "heading")
    }


def _actual(value: Any, path: str) -> dict[str, float]:
    obj = _exact_keys(value, ("linear_velocity", "angular_velocity"), path)
    return {
        key: _finite_number(obj[key], f"{path}.{key}")
        for key in ("linear_velocity", "angular_velocity")
    }


def _liquid(value: Any, path: str) -> dict[str, float]:
    keys = ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot")
    obj = _exact_keys(value, keys, path)
    return {key: _finite_number(obj[key], f"{path}.{key}") for key in keys}


def _physical(value: Any, path: str) -> dict[str, Any]:
    obj = _exact_keys(value, ("pose", "actual", "liquid"), path)
    return {
        "pose": _pose(obj["pose"], f"{path}.pose"),
        "actual": _actual(obj["actual"], f"{path}.actual"),
        "liquid": _liquid(obj["liquid"], f"{path}.liquid"),
    }


def _publisher(value: Any, path: str) -> dict[str, float]:
    keys = (
        "previous_linear_command",
        "previous_angular_command",
        "previous_linear_acceleration",
        "previous_angular_acceleration",
    )
    obj = _exact_keys(value, keys, path)
    return {key: _finite_number(obj[key], f"{path}.{key}") for key in keys}


def _state(
    value: Any,
    path: str,
    linear_older_count: int,
    angular_older_count: int,
) -> dict[str, Any]:
    obj = _exact_keys(
        value,
        ("physical", "progress", "publisher", "linear_older", "angular_older"),
        path,
    )
    return {
        "physical": _physical(obj["physical"], f"{path}.physical"),
        "progress": _finite_number(obj["progress"], f"{path}.progress"),
        "publisher": _publisher(obj["publisher"], f"{path}.publisher"),
        "linear_older": _numeric_array(
            obj["linear_older"], linear_older_count, f"{path}.linear_older"
        ),
        "angular_older": _numeric_array(
            obj["angular_older"], angular_older_count, f"{path}.angular_older"
        ),
    }


def _channel(value: Any, path: str) -> dict[str, float]:
    obj = _exact_keys(value, ("tau_sec", "gain"), path)
    tau = _finite_number(obj["tau_sec"], f"{path}.tau_sec")
    gain = _finite_number(obj["gain"], f"{path}.gain")
    if tau <= 0.0 or gain <= 0.0:
        raise FixtureError(f"{path} tau_sec and gain must be positive")
    return {"tau_sec": tau, "gain": gain}


def _validate_scenario(value: Any, index: int) -> dict[str, Any]:
    path = f"scenarios[{index}]"
    obj = _exact_keys(
        value, ("id", "config", "plant", "state", "control", "expected"), path
    )
    scenario_id = _string(obj["id"], f"{path}.id")
    if scenario_id != "MatchesIndependentCompleteMapGolden":
        raise FixtureError(f"{path}.id is not the frozen Stage 2e scenario")

    config_obj = _exact_keys(
        obj["config"],
        (
            "dt_sec",
            "maximum_linear_delay_sec",
            "maximum_angular_delay_sec",
            "linear_delay_sec",
            "angular_delay_sec",
            "integer_snap_tolerance_ratio",
            "duration_tolerance_sec",
        ),
        f"{path}.config",
    )
    config = {
        key: _finite_number(config_obj[key], f"{path}.config.{key}")
        for key in (
            "dt_sec",
            "maximum_linear_delay_sec",
            "maximum_angular_delay_sec",
            "linear_delay_sec",
            "angular_delay_sec",
            "integer_snap_tolerance_ratio",
            "duration_tolerance_sec",
        )
    }
    if (
        config["dt_sec"] <= 0.0
        or config["maximum_linear_delay_sec"] < 0.0
        or config["maximum_angular_delay_sec"] < 0.0
    ):
        raise FixtureError(f"{path}.config has invalid dt or maximum delay")
    if config["linear_delay_sec"] < 0.0 or config["angular_delay_sec"] < 0.0:
        raise FixtureError(f"{path}.config has invalid delay")
    if (
        config["linear_delay_sec"] > config["maximum_linear_delay_sec"]
        or config["angular_delay_sec"] > config["maximum_angular_delay_sec"]
    ):
        raise FixtureError(f"{path}.config delay exceeds maximum delay")
    if not 0.0 <= config["integer_snap_tolerance_ratio"] < 0.5:
        raise FixtureError(f"{path}.config integer snap tolerance is invalid")
    if not 0.0 <= config["duration_tolerance_sec"] < config["dt_sec"]:
        raise FixtureError(f"{path}.config duration tolerance is invalid")
    linear_dimensions = delay_dimensions(
        config["maximum_linear_delay_sec"], config["dt_sec"]
    )
    angular_dimensions = delay_dimensions(
        config["maximum_angular_delay_sec"], config["dt_sec"]
    )

    plant_obj = _exact_keys(
        obj["plant"], ("linear_actuator", "angular_actuator", "liquid"), f"{path}.plant"
    )
    liquid_obj = _exact_keys(
        plant_obj["liquid"],
        (
            "natural_frequency_rad_per_sec",
            "damping_ratio",
            "longitudinal_coupling",
            "lateral_coupling",
        ),
        f"{path}.plant.liquid",
    )
    liquid = {
        key: _finite_number(liquid_obj[key], f"{path}.plant.liquid.{key}")
        for key in (
            "natural_frequency_rad_per_sec",
            "damping_ratio",
            "longitudinal_coupling",
            "lateral_coupling",
        )
    }
    if (
        liquid["natural_frequency_rad_per_sec"] <= 0.0
        or liquid["damping_ratio"] < 0.0
        or liquid["longitudinal_coupling"] <= 0.0
        or liquid["lateral_coupling"] <= 0.0
    ):
        raise FixtureError(f"{path}.plant.liquid has invalid parameters")
    plant = {
        "linear_actuator": _channel(
            plant_obj["linear_actuator"], f"{path}.plant.linear_actuator"
        ),
        "angular_actuator": _channel(
            plant_obj["angular_actuator"], f"{path}.plant.angular_actuator"
        ),
        "liquid": liquid,
    }

    control_obj = _exact_keys(
        obj["control"],
        ("linear_jerk", "angular_jerk", "progress_velocity"),
        f"{path}.control",
    )
    control = {
        key: _finite_number(control_obj[key], f"{path}.control.{key}")
        for key in ("linear_jerk", "angular_jerk", "progress_velocity")
    }
    if control["progress_velocity"] < 0.0:
        raise FixtureError(f"{path}.control.progress_velocity must be nonnegative")

    expected_obj = _exact_keys(
        obj["expected"], ("issued", "segments", "next_state"), f"{path}.expected"
    )
    issued_obj = _exact_keys(
        expected_obj["issued"],
        (
            "linear_command",
            "angular_command",
            "linear_acceleration",
            "angular_acceleration",
        ),
        f"{path}.expected.issued",
    )
    issued = {
        key: _finite_number(issued_obj[key], f"{path}.expected.issued.{key}")
        for key in issued_obj
    }
    segments: list[dict[str, float]] = []
    for segment_index, segment_value in enumerate(
        _array(expected_obj["segments"], 3, f"{path}.expected.segments")
    ):
        segment_obj = _exact_keys(
            segment_value,
            ("duration_sec", "linear_target", "angular_target"),
            f"{path}.expected.segments[{segment_index}]",
        )
        segments.append(
            {
                key: _finite_number(
                    segment_obj[key], f"{path}.expected.segments[{segment_index}].{key}"
                )
                for key in segment_obj
            }
        )
    return {
        "id": scenario_id,
        "config": config,
        "plant": plant,
        "state": _state(
            obj["state"],
            f"{path}.state",
            linear_dimensions["older_count"],
            angular_dimensions["older_count"],
        ),
        "control": control,
        "expected": {
            "issued": issued,
            "segments": segments,
            "next_state": _state(
                expected_obj["next_state"],
                f"{path}.expected.next_state",
                linear_dimensions["older_count"],
                angular_dimensions["older_count"],
            ),
        },
    }


def load_fixture(path: Path | str) -> dict[str, Any]:
    """Load and strictly validate a Stage 2e fixture."""
    fixture_path = Path(path)
    try:
        text = fixture_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise FixtureError(f"cannot read fixture {fixture_path}: {exc}") from exc
    try:
        value = json.loads(
            text,
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonfinite_constant,
        )
    except FixtureError:
        raise
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise FixtureError(f"invalid JSON in {fixture_path}: {exc}") from exc
    root = _exact_keys(value, ("schema_version", "numeric", "scenarios"), "fixture")
    if root["schema_version"] != "stage2_execution_golden_v1":
        raise FixtureError("fixture.schema_version must be stage2_execution_golden_v1")
    numeric = _exact_keys(root["numeric"], ("absolute_tolerance",), "fixture.numeric")
    absolute_tolerance = _finite_number(
        numeric["absolute_tolerance"], "fixture.numeric.absolute_tolerance"
    )
    if absolute_tolerance <= 0.0:
        raise FixtureError("fixture.numeric.absolute_tolerance must be positive")
    scenarios_value = _array(root["scenarios"], 1, "fixture.scenarios")
    scenario = _validate_scenario(scenarios_value[0], 0)
    return {
        "schema_version": root["schema_version"],
        "numeric": {"absolute_tolerance": absolute_tolerance},
        "scenarios": [scenario],
    }


def select_scenario(
    fixture: Mapping[str, Any], scenario_id: str = "MatchesIndependentCompleteMapGolden"
) -> Mapping[str, Any]:
    """Select the one scenario frozen by the v1 fixture schema."""
    matches = [
        scenario for scenario in fixture["scenarios"] if scenario["id"] == scenario_id
    ]
    if len(matches) != 1:
        raise FixtureError(
            f"fixture must contain exactly one scenario named {scenario_id}"
        )
    return matches[0]


def canonical_json(value: Mapping[str, Any]) -> str:
    """Return the deterministic JSON representation hashed into the header."""
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    )


def canonical_sha256(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def _issue(
    state: Mapping[str, Any], control: Mapping[str, float], dt: float
) -> dict[str, float]:
    publisher = state["publisher"]
    half_dt_squared = 0.5 * dt * dt
    return {
        "linear_acceleration": publisher["previous_linear_acceleration"]
        + dt * control["linear_jerk"],
        "angular_acceleration": publisher["previous_angular_acceleration"]
        + dt * control["angular_jerk"],
        "linear_command": publisher["previous_linear_command"]
        + dt * publisher["previous_linear_acceleration"]
        + half_dt_squared * control["linear_jerk"],
        "angular_command": publisher["previous_angular_command"]
        + dt * publisher["previous_angular_acceleration"]
        + half_dt_squared * control["angular_jerk"],
    }


def _channel_schedule(
    delay: float, dt: float, snap: float, width: int = 4
) -> dict[str, Any]:
    ratio = delay / dt
    lower = math.floor(ratio)
    beta = ratio - lower
    if beta <= snap:
        beta = 0.0
    elif 1.0 - beta <= snap:
        beta = 0.0
        lower += 1
    if lower < 0 or lower >= width or (beta > 0.0 and lower + 1 >= width):
        raise FixtureError("delay selector index exceeds frozen width")
    first = 0 if beta == 0.0 else lower + 1
    return {
        "duration": [beta * dt, dt - beta * dt, 0.0],
        "selector": [first, lower, 0],
        "integer_delay_steps": lower,
        "fractional_beta": beta,
    }


def _merge_schedules(
    linear: Mapping[str, Any], angular: Mapping[str, Any], dt: float
) -> list[dict[str, Any]]:
    boundaries = sorted({0.0, linear["duration"][0], angular["duration"][0], dt})
    if len(boundaries) > 4 or boundaries[0] != 0.0 or boundaries[-1] != dt:
        raise FixtureError("invalid delay schedule union")
    segments: list[dict[str, Any]] = []
    accumulated_duration = 0.0
    active_count = len(boundaries) - 1
    for index in range(active_count):
        left = boundaries[index]
        right = boundaries[index + 1]
        if not right > left:
            raise FixtureError("delay schedule contains an empty segment")
        duration = (
            dt - accumulated_duration if index + 1 == active_count else right - left
        )
        if not math.isfinite(duration) or duration <= 0.0:
            raise FixtureError("delay schedule duration is invalid")
        linear_selector = (
            linear["selector"][0]
            if left < linear["duration"][0]
            else linear["selector"][1]
        )
        angular_selector = (
            angular["selector"][0]
            if left < angular["duration"][0]
            else angular["selector"][1]
        )
        segments.append(
            {
                "duration_sec": duration,
                "linear_selector": linear_selector,
                "angular_selector": angular_selector,
            }
        )
        accumulated_duration += duration
    while len(segments) < 3:
        segments.append(
            {"duration_sec": 0.0, "linear_selector": 0, "angular_selector": 0}
        )
    return segments


def _fopdt(
    actual: float, target: float, duration: float, channel: Mapping[str, float]
) -> float:
    if duration == 0.0:
        return actual
    exponent = duration / channel["tau_sec"]
    rho = math.exp(-exponent)
    one_minus_rho = -math.expm1(-exponent)
    return rho * actual + one_minus_rho * channel["gain"] * target


def _actual_at_elapsed(
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
    actual = _actual_at_elapsed(initial_actual, segment, plant, elapsed)
    liquid_params = plant["liquid"]
    damping = (
        2.0
        * liquid_params["damping_ratio"]
        * liquid_params["natural_frequency_rad_per_sec"]
    )
    stiffness = liquid_params["natural_frequency_rad_per_sec"] ** 2
    longitudinal_acceleration = (
        plant["linear_actuator"]["gain"] * segment["linear_target"]
        - actual["linear_velocity"]
    ) / plant["linear_actuator"]["tau_sec"]
    lateral_acceleration = actual["linear_velocity"] * actual["angular_velocity"]
    return [
        liquid["eta_x_dot"],
        -damping * liquid["eta_x_dot"]
        - stiffness * liquid["eta_x"]
        - liquid_params["longitudinal_coupling"] * longitudinal_acceleration,
        liquid["eta_y_dot"],
        -damping * liquid["eta_y_dot"]
        - stiffness * liquid["eta_y"]
        - liquid_params["lateral_coupling"] * lateral_acceleration,
    ]


def _liquid_add_scaled(
    initial: Mapping[str, float], derivative: Sequence[float], scale: float
) -> dict[str, float]:
    return {
        "eta_x": initial["eta_x"] + scale * derivative[0],
        "eta_x_dot": initial["eta_x_dot"] + scale * derivative[1],
        "eta_y": initial["eta_y"] + scale * derivative[2],
        "eta_y_dot": initial["eta_y_dot"] + scale * derivative[3],
    }


def _propagate_segment(
    physical: Mapping[str, Any], segment: Mapping[str, float], plant: Mapping[str, Any]
) -> dict[str, Any]:
    duration = segment["duration_sec"]
    if duration == 0.0:
        return {
            "pose": dict(physical["pose"]),
            "actual": dict(physical["actual"]),
            "liquid": dict(physical["liquid"]),
        }
    midpoint_actual = _actual_at_elapsed(
        physical["actual"], segment, plant, 0.5 * duration
    )
    end_actual = _actual_at_elapsed(physical["actual"], segment, plant, duration)
    liquid = physical["liquid"]
    half_duration = 0.5 * duration
    k1 = _liquid_derivative(liquid, physical["actual"], segment, plant, 0.0)
    intermediate = _liquid_add_scaled(liquid, k1, half_duration)
    k2 = _liquid_derivative(
        intermediate, physical["actual"], segment, plant, half_duration
    )
    intermediate = _liquid_add_scaled(liquid, k2, half_duration)
    k3 = _liquid_derivative(
        intermediate, physical["actual"], segment, plant, half_duration
    )
    intermediate = _liquid_add_scaled(liquid, k3, duration)
    k4 = _liquid_derivative(intermediate, physical["actual"], segment, plant, duration)
    scale = duration / 6.0
    next_liquid = {
        "eta_x": liquid["eta_x"] + scale * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
        "eta_x_dot": liquid["eta_x_dot"]
        + scale * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]),
        "eta_y": liquid["eta_y"] + scale * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]),
        "eta_y_dot": liquid["eta_y_dot"]
        + scale * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]),
    }
    heading_delta = duration * midpoint_actual["angular_velocity"]
    heading_midpoint = physical["pose"]["heading"] + 0.5 * heading_delta
    return {
        "pose": {
            "x": physical["pose"]["x"]
            + duration
            * midpoint_actual["linear_velocity"]
            * math.cos(heading_midpoint),
            "y": physical["pose"]["y"]
            + duration
            * midpoint_actual["linear_velocity"]
            * math.sin(heading_midpoint),
            "heading": physical["pose"]["heading"] + heading_delta,
        },
        "actual": end_actual,
        "liquid": next_liquid,
    }


def calculate(scenario: Mapping[str, Any]) -> dict[str, Any]:
    """Recompute issue, delay union, plant and queue shift for one scenario."""
    config = scenario["config"]
    state = scenario["state"]
    plant = scenario["plant"]
    control = scenario["control"]
    linear_dimensions = delay_dimensions(
        config["maximum_linear_delay_sec"], config["dt_sec"]
    )
    angular_dimensions = delay_dimensions(
        config["maximum_angular_delay_sec"], config["dt_sec"]
    )
    issued = _issue(state, control, config["dt_sec"])
    linear_schedule = _channel_schedule(
        config["linear_delay_sec"],
        config["dt_sec"],
        config["integer_snap_tolerance_ratio"],
        linear_dimensions["selector_width"],
    )
    angular_schedule = _channel_schedule(
        config["angular_delay_sec"],
        config["dt_sec"],
        config["integer_snap_tolerance_ratio"],
        angular_dimensions["selector_width"],
    )
    schedule = _merge_schedules(linear_schedule, angular_schedule, config["dt_sec"])
    linear_taps = [
        issued["linear_command"],
        state["publisher"]["previous_linear_command"],
        *state["linear_older"],
    ][: linear_dimensions["selector_width"]]
    angular_taps = [
        issued["angular_command"],
        state["publisher"]["previous_angular_command"],
        *state["angular_older"],
    ][: angular_dimensions["selector_width"]]
    segments = [
        {
            "duration_sec": item["duration_sec"],
            "linear_target": linear_taps[item["linear_selector"]],
            "angular_target": angular_taps[item["angular_selector"]],
        }
        for item in schedule
    ]
    physical = state["physical"]
    for segment in segments:
        physical = _propagate_segment(physical, segment, plant)
    progress = state["progress"]
    for segment in segments:
        progress += segment["duration_sec"] * control["progress_velocity"]
        if not math.isfinite(progress):
            raise FixtureError("progress propagation produced a non-finite value")
    next_state = {
        "physical": physical,
        "progress": progress,
        "publisher": {
            "previous_linear_command": issued["linear_command"],
            "previous_angular_command": issued["angular_command"],
            "previous_linear_acceleration": issued["linear_acceleration"],
            "previous_angular_acceleration": issued["angular_acceleration"],
        },
        "linear_older": (
            [state["publisher"]["previous_linear_command"]]
            + list(state["linear_older"])
        )[: linear_dimensions["older_count"]],
        "angular_older": (
            [state["publisher"]["previous_angular_command"]]
            + list(state["angular_older"])
        )[: angular_dimensions["older_count"]],
    }
    return {"issued": issued, "segments": segments, "next_state": next_state}


def compare_expected(scenario: Mapping[str, Any], *, absolute_tolerance: float) -> None:
    """Raise FixtureError if the hand-authored expected map drifts."""
    actual = calculate(scenario)
    expected = scenario["expected"]

    def compare(left: Any, right: Any, path: str) -> None:
        if isinstance(left, Mapping):
            if set(left) != set(right):
                raise FixtureError(f"{path} key set differs from reference")
            for key in left:
                compare(left[key], right[key], f"{path}.{key}")
        elif isinstance(left, list):
            if len(left) != len(right):
                raise FixtureError(f"{path} array length differs from reference")
            for index, (item_left, item_right) in enumerate(zip(left, right)):
                compare(item_left, item_right, f"{path}[{index}]")
        else:
            if not math.isclose(
                float(left), float(right), rel_tol=0.0, abs_tol=absolute_tolerance
            ):
                raise FixtureError(
                    f"{path} differs: expected {left!r}, reference {right!r}"
                )

    compare(expected, actual, f"scenarios[{scenario['id']}].expected")


def validate_expected(fixture: Mapping[str, Any]) -> None:
    """Recompute the fixture with its explicit frozen numeric tolerance."""
    compare_expected(
        select_scenario(fixture),
        absolute_tolerance=fixture["numeric"]["absolute_tolerance"],
    )
