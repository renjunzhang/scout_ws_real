#!/usr/bin/env python3
"""Generate a build-only C++14 header from the Stage 2e JSON fixture."""

from __future__ import annotations

import argparse
import math
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

try:
    from stage2_execution_golden_reference import (
        FixtureError,
        canonical_sha256,
        delay_dimensions,
        load_fixture,
        select_scenario,
        validate_expected,
    )
except ImportError:  # pragma: no cover - only used when imported unusually
    from .stage2_execution_golden_reference import (  # type: ignore
        FixtureError,
        canonical_sha256,
        delay_dimensions,
        load_fixture,
        select_scenario,
        validate_expected,
    )


def _cpp_number(value: Any) -> str:
    number = float(value)
    if not math.isfinite(number):
        raise FixtureError("cannot emit a non-finite C++ number")
    # repr is deterministic on supported Python versions and emits a C++14
    # compatible decimal/exponent spelling for finite IEEE-754 values.
    text = repr(number)
    if text == "-0.0":
        return "-0.0"
    return text


def _state_lines(name: str, state: Mapping[str, Any]) -> str:
    physical = state["physical"]
    pose = physical["pose"]
    actual = physical["actual"]
    liquid = physical["liquid"]
    publisher = state["publisher"]
    linear_older = ", ".join(_cpp_number(value) for value in state["linear_older"])
    angular_older = ", ".join(_cpp_number(value) for value in state["angular_older"])
    return "\n".join(
        [
            f"static const GoldenState {name} = {{",
            f"    {{{{{_cpp_number(pose['x'])}, {_cpp_number(pose['y'])}, {_cpp_number(pose['heading'])}}}, {{{_cpp_number(actual['linear_velocity'])}, {_cpp_number(actual['angular_velocity'])}}}, {{{_cpp_number(liquid['eta_x'])}, {_cpp_number(liquid['eta_x_dot'])}, {_cpp_number(liquid['eta_y'])}, {_cpp_number(liquid['eta_y_dot'])}}}}},",
            f"    {_cpp_number(state['progress'])},",
            f"    {{{_cpp_number(publisher['previous_linear_command'])}, {_cpp_number(publisher['previous_angular_command'])}, {_cpp_number(publisher['previous_linear_acceleration'])}, {_cpp_number(publisher['previous_angular_acceleration'])}}},",
            f"    {{{linear_older}}},",
            f"    {{{angular_older}}}",
            "};",
        ]
    )


def render_header(fixture: Mapping[str, Any]) -> str:
    scenario = select_scenario(fixture)
    config = scenario["config"]
    plant = scenario["plant"]
    liquid = plant["liquid"]
    state = scenario["state"]
    expected = scenario["expected"]
    issued = expected["issued"]
    digest = canonical_sha256(fixture)
    linear_dimensions = delay_dimensions(
        config["maximum_linear_delay_sec"], config["dt_sec"]
    )
    angular_dimensions = delay_dimensions(
        config["maximum_angular_delay_sec"], config["dt_sec"]
    )
    lines = [
        "// Generated from test/fixtures/stage2_execution_golden_v1.json.",
        "// Build-only test artifact; do not install or include from production code.",
        "#pragma once",
        "",
        "#include <array>",
        "#include <cstddef>",
        "",
        "namespace spmpc_local_planner {",
        "namespace mainline {",
        "namespace stage2_execution_golden {",
        "",
        f'static constexpr char kSchemaVersion[] = "{fixture["schema_version"]}";',
        f'static constexpr char kScenarioId[] = "{scenario["id"]}";',
        f'static constexpr char kCanonicalJsonSha256[] = "{digest}";',
        f"static constexpr double kAbsoluteTolerance = {_cpp_number(fixture['numeric']['absolute_tolerance'])};",
        f"static constexpr std::size_t kLinearSelectorWidth = {linear_dimensions['selector_width']};",
        f"static constexpr std::size_t kAngularSelectorWidth = {angular_dimensions['selector_width']};",
        f"static constexpr std::size_t kLinearOlderCount = {linear_dimensions['older_count']};",
        f"static constexpr std::size_t kAngularOlderCount = {angular_dimensions['older_count']};",
        f"static constexpr std::size_t kSegmentCount = {len(expected['segments'])};",
        "",
        "struct GoldenConfig {",
        "  double dt_sec;",
        "  double maximum_linear_delay_sec;",
        "  double maximum_angular_delay_sec;",
        "  double linear_delay_sec;",
        "  double angular_delay_sec;",
        "  double integer_snap_tolerance_ratio;",
        "  double duration_tolerance_sec;",
        "};",
        "struct GoldenChannel { double tau_sec; double gain; };",
        "struct GoldenLiquidParams {",
        "  double natural_frequency_rad_per_sec;",
        "  double damping_ratio;",
        "  double longitudinal_coupling;",
        "  double lateral_coupling;",
        "};",
        "struct GoldenPhysical {",
        "  struct { double x; double y; double heading; } pose;",
        "  struct { double linear_velocity; double angular_velocity; } actual;",
        "  struct { double eta_x; double eta_x_dot; double eta_y; double eta_y_dot; } liquid;",
        "};",
        "struct GoldenPublisher {",
        "  double previous_linear_command;",
        "  double previous_angular_command;",
        "  double previous_linear_acceleration;",
        "  double previous_angular_acceleration;",
        "};",
        "struct GoldenState {",
        "  GoldenPhysical physical;",
        "  double progress;",
        "  GoldenPublisher publisher;",
        "  std::array<double, kLinearOlderCount> linear_older;",
        "  std::array<double, kAngularOlderCount> angular_older;",
        "};",
        "struct GoldenControl { double linear_jerk; double angular_jerk; double progress_velocity; };",
        "struct GoldenIssued {",
        "  double linear_command;",
        "  double angular_command;",
        "  double linear_acceleration;",
        "  double angular_acceleration;",
        "};",
        "struct GoldenSegment { double duration_sec; double linear_target; double angular_target; };",
        "",
        "static const GoldenConfig kConfig = {",
        f"    {_cpp_number(config['dt_sec'])}, {_cpp_number(config['maximum_linear_delay_sec'])}, {_cpp_number(config['maximum_angular_delay_sec'])},",
        f"    {_cpp_number(config['linear_delay_sec'])}, {_cpp_number(config['angular_delay_sec'])}, {_cpp_number(config['integer_snap_tolerance_ratio'])}, {_cpp_number(config['duration_tolerance_sec'])}",
        "};",
        "static const GoldenChannel kLinearActuator = {",
        f"    {_cpp_number(plant['linear_actuator']['tau_sec'])}, {_cpp_number(plant['linear_actuator']['gain'])}}};",
        "static const GoldenChannel kAngularActuator = {",
        f"    {_cpp_number(plant['angular_actuator']['tau_sec'])}, {_cpp_number(plant['angular_actuator']['gain'])}}};",
        "static const GoldenLiquidParams kLiquid = {",
        f"    {_cpp_number(liquid['natural_frequency_rad_per_sec'])}, {_cpp_number(liquid['damping_ratio'])}, {_cpp_number(liquid['longitudinal_coupling'])}, {_cpp_number(liquid['lateral_coupling'])}}};",
        _state_lines("kInitialState", state),
        "static const GoldenControl kControl = {",
        f"    {_cpp_number(scenario['control']['linear_jerk'])}, {_cpp_number(scenario['control']['angular_jerk'])}, {_cpp_number(scenario['control']['progress_velocity'])}}};",
        "static const GoldenIssued kExpectedIssued = {",
        f"    {_cpp_number(issued['linear_command'])}, {_cpp_number(issued['angular_command'])}, {_cpp_number(issued['linear_acceleration'])}, {_cpp_number(issued['angular_acceleration'])}}};",
        "static const std::array<GoldenSegment, kSegmentCount> kExpectedSegments = {{",
    ]
    for segment in expected["segments"]:
        lines.append(
            f"    GoldenSegment{{{_cpp_number(segment['duration_sec'])}, {_cpp_number(segment['linear_target'])}, {_cpp_number(segment['angular_target'])}}},"
        )
    lines.extend(
        [
            "}};",
            _state_lines("kExpectedNextState", expected["next_state"]),
            "",
            "}  // namespace stage2_execution_golden",
            "}  // namespace mainline",
            "}  // namespace spmpc_local_planner",
            "",
        ]
    )
    return "\n".join(lines)


def generate(fixture_path: Path, output_path: Path, *, check: bool = False) -> None:
    fixture = load_fixture(fixture_path)
    validate_expected(fixture)
    rendered = render_header(fixture)
    if check:
        try:
            current = output_path.read_text(encoding="utf-8")
        except OSError as exc:
            raise FixtureError(
                f"cannot read generated header for --check: {exc}"
            ) from exc
        if current != rendered:
            raise FixtureError(
                "generated header is stale; regenerate it from the fixture"
            )
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(rendered, encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify an existing header is exactly reproducible without writing it",
    )
    args = parser.parse_args(argv)
    try:
        generate(args.fixture, args.output, check=args.check)
    except (FixtureError, OSError) as exc:
        print(f"stage2 golden generation failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
