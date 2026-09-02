#!/usr/bin/env python3
"""Generate a build-tree C++14 header for the Stage 2f/2g golden fixture."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

try:
    import stage2_fg_reference as reference
except ImportError:  # pragma: no cover - package-style imports only
    from . import stage2_fg_reference as reference  # type: ignore


def _cpp_number(value: Any) -> str:
    number = float(value)
    if not math.isfinite(number):
        raise reference.FixtureError("cannot emit a non-finite C++ number")
    return repr(number)


def _cpp_string(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def _validate_integer_ranges(value: Any, path: str = "fixture") -> None:
    if isinstance(value, bool):
        return
    if isinstance(value, int):
        if value < -(1 << 63) or value > (1 << 64) - 1:
            raise reference.FixtureError(
                f"{path} integer is outside C++ int64/uint64 range"
            )
        return
    if isinstance(value, Mapping):
        for key, item in value.items():
            _validate_integer_ranges(item, f"{path}.{key}")
        return
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for index, item in enumerate(value):
            _validate_integer_ranges(item, f"{path}[{index}]")


def _aggregate(type_name: str, values: Sequence[str]) -> str:
    return f"{type_name}{{{', '.join(values)}}}"


def _physical(value: Mapping[str, Any]) -> str:
    pose = value["pose"]
    actual = value["actual"]
    liquid = value["liquid"]
    return _aggregate(
        "GoldenPhysical",
        [
            _aggregate(
                "GoldenPose", [_cpp_number(pose[key]) for key in ("x", "y", "heading")]
            ),
            _aggregate(
                "GoldenActual",
                [
                    _cpp_number(actual[key])
                    for key in ("linear_velocity", "angular_velocity")
                ],
            ),
            _aggregate(
                "GoldenLiquid",
                [
                    _cpp_number(liquid[key])
                    for key in ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot")
                ],
            ),
        ],
    )


def _publisher(value: Mapping[str, Any]) -> str:
    return _aggregate(
        "GoldenPublisher",
        [
            _cpp_number(value[key])
            for key in (
                "previous_linear_command",
                "previous_angular_command",
                "previous_linear_acceleration",
                "previous_angular_acceleration",
            )
        ],
    )


def _config(value: Mapping[str, Any]) -> str:
    keys = (
        "dt_sec",
        "maximum_linear_delay_sec",
        "maximum_angular_delay_sec",
        "linear_delay_sec",
        "angular_delay_sec",
        "integer_snap_tolerance_ratio",
        "duration_tolerance_sec",
    )
    return _aggregate("GoldenActuatorConfig", [_cpp_number(value[key]) for key in keys])


def _plant(value: Mapping[str, Any]) -> str:
    linear, angular, liquid = (
        value["linear_actuator"],
        value["angular_actuator"],
        value["liquid"],
    )
    return _aggregate(
        "GoldenPlant",
        [
            "{"
            + ", ".join(_cpp_number(linear[key]) for key in ("tau_sec", "gain"))
            + "}",
            "{"
            + ", ".join(_cpp_number(angular[key]) for key in ("tau_sec", "gain"))
            + "}",
            "{"
            + ", ".join(
                _cpp_number(liquid[key])
                for key in (
                    "natural_frequency_rad_per_sec",
                    "damping_ratio",
                    "longitudinal_coupling",
                    "lateral_coupling",
                )
            )
            + "}",
        ],
    )


def _known_case(index: int, case: Mapping[str, Any]) -> list[str]:
    expected = case["expected"]
    state = expected["state"]
    config = case["config"]
    linear_count = math.ceil(config["maximum_linear_delay_sec"] / config["dt_sec"])
    angular_count = math.ceil(config["maximum_angular_delay_sec"] / config["dt_sec"])
    linear_selector_width = linear_count + 1
    angular_selector_width = angular_count + 1
    linear_older_count = max(0, linear_count - 1)
    angular_older_count = max(0, angular_count - 1)
    segment_count = len(expected["segments"])
    lines = [
        f"struct GoldenPrefixExpected{index} {{",
        f"  static constexpr std::size_t kLinearOlderCount = {linear_older_count};",
        f"  static constexpr std::size_t kAngularOlderCount = {angular_older_count};",
        f"  static constexpr std::size_t kSegmentCount = {segment_count};",
        "  const char* status;",
        "  GoldenPhysical physical;",
        "  GoldenPublisher publisher;",
        "  std::array<double, kLinearOlderCount> linear_older;",
        "  std::array<double, kAngularOlderCount> angular_older;",
        "  std::array<GoldenTargetSegment, kSegmentCount> segments;",
        "  std::size_t segment_count;",
        "  std::uint64_t history_generation;",
        "  std::uint64_t last_emitted_cycle_id;",
        "  std::int64_t start_model_ns;",
        "  std::int64_t target_model_ns;",
        "  GoldenCoverage coverage;",
        "};",
        f"struct GoldenKnownPrefixCase{index} {{",
        f"  static constexpr std::size_t kHistoryCapacity = {case['history_capacity']};",
        f"  static constexpr std::size_t kHistoryCount = {len(case['history'])};",
        f"  static constexpr std::size_t kLinearSelectorWidth = {linear_selector_width};",
        f"  static constexpr std::size_t kAngularSelectorWidth = {angular_selector_width};",
        "  const char* id;",
        "  std::int64_t maximum_history_gap_ns;",
        "  std::int64_t maximum_publish_lateness_ns;",
        "  std::int64_t snapshot_lateness_ns;",
        "  std::uint64_t reset_epoch;",
        "  std::int64_t anchor_steady_ns;",
        "  std::int64_t anchor_model_ns;",
        "  GoldenActuatorConfig config;",
        "  GoldenPlant plant;",
        "  std::int64_t start_model_ns;",
        "  std::uint64_t target_cycle_id;",
        "  GoldenPhysical initial_physical;",
        "  std::array<GoldenPrefixEvent, kHistoryCount> history;",
        f"  GoldenPrefixExpected{index} expected;",
        "};",
        f"static const GoldenKnownPrefixCase{index} kKnownPrefixCase{index} = {{",
        f"  {_cpp_string(case['id'])}, {case['maximum_history_gap_ns']}, {case['maximum_publish_lateness_ns']}, {case['snapshot_lateness_ns']}, {case['reset_epoch']},",
        f"  {case['clock']['anchor_steady_ns']}, {case['clock']['anchor_model_ns']},",
        f"  {_config(case['config'])}, {_plant(case['plant'])},",
        f"  {case['start_model_ns']}, {case['target_cycle_id']}, {_physical(case['initial_physical'])},",
        "  {{",
    ]
    for event in case["history"]:
        lines.append(
            "    GoldenPrefixEvent{"
            f"{event['cycle_id']}, {event['release_generation']}, "
            f"{_cpp_string(event['emission_reason'])}, "
            f"{_cpp_number(event['linear_command'])}, {_cpp_number(event['angular_command'])}, "
            f"{_cpp_number(event['linear_acceleration'])}, {_cpp_number(event['angular_acceleration'])}, "
            f"{event['actual_lateness_ns']}"
            "},"
        )
    lines.extend(
        [
            "  }},",
            f"  GoldenPrefixExpected{index}{{",
            f"    {_cpp_string(expected['status'])}, {_physical(state['physical'])}, {_publisher(state['publisher'])},",
            "    {{"
            + ", ".join(_cpp_number(item) for item in state["linear_older"])
            + "}},",
            "    {{"
            + ", ".join(_cpp_number(item) for item in state["angular_older"])
            + "}},",
            "    {{",
        ]
    )
    for segment in expected["segments"]:
        lines.append(
            "      GoldenTargetSegment{"
            f"{_cpp_number(segment['duration_sec'])}, {_cpp_number(segment['linear_target'])}, "
            f"{_cpp_number(segment['angular_target'])}"
            "},"
        )
    coverage = expected["coverage"]
    lines.extend(
        [
            "    }},",
            f"    {expected['segment_count']}, {expected['history_generation']}, {expected['last_emitted_cycle_id']},",
            f"    {expected['start_model_ns']}, {expected['target_model_ns']},",
            (
                f"    GoldenCoverage{{{_cpp_string(coverage['status'])}, "
                f"{coverage['history_generation']}, "
                f"{coverage['predecessor_release_generation']}, "
                f"{coverage['covered_event_count']}, "
                f"{coverage['maximum_adjacent_gap_ns']}, "
                f"{coverage['future_hold_ns']}, "
                f"{coverage['maximum_required_gap_ns']}}}"
            ),
            "  }",
            "};",
            "",
        ]
    )
    return lines


def _hash_initializer(path_hash: str) -> str:
    values = [f"0x{path_hash[index : index + 2]}" for index in range(0, 64, 2)]
    return "{{" + ", ".join(values) + "}}"


def _projection_case(index: int, case: Mapping[str, Any]) -> list[str]:
    path = case["path"]
    identity = path["identity"]
    authority = case["authority"]
    expected = case["expected"]
    lines = [
        f"struct GoldenNominalProjectionCase{index} {{",
        f"  static constexpr std::size_t kVertexCapacity = {len(path['vertices'])};",
        f"  static constexpr std::size_t kVertexCount = {len(path['vertices'])};",
        "  const char* id;",
        "  const char* source_known_prefix_case_id;",
        "  GoldenProjectorConfig config;",
        "  GoldenPathIdentity identity;",
        "  std::array<GoldenPathVertex, kVertexCount> vertices;",
        "  double s_path_end;",
        "  GoldenNominalAuthority authority;",
        "  GoldenProjectionExpected expected;",
        "};",
        f"static const GoldenNominalProjectionCase{index} kNominalProjectionCase{index} = {{",
        f"  {_cpp_string(case['id'])}, {_cpp_string(case['source_known_prefix_case_id'])},",
        "  GoldenProjectorConfig{"
        + ", ".join(
            _cpp_number(case["config"][key])
            for key in (
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
        )
        + "},",
        f"  GoldenPathIdentity{{{identity['path_id']}, {_hash_initializer(identity['path_hash'])}, {identity['reset_epoch']}}},",
        "  {{",
    ]
    for vertex in path["vertices"]:
        lines.append(
            f"    GoldenPathVertex{{{_cpp_number(vertex['x'])}, {_cpp_number(vertex['y'])}, {_cpp_number(vertex['cumulative_s'])}}},"
        )
    lines.extend(
        [
            "  }},",
            f"  {_cpp_number(path['s_path_end'])},",
            (
                f"  GoldenNominalAuthority{{{_cpp_string(authority['kind'])}, "
                f"GoldenPathIdentity{{"
                f"{authority['identity']['path_id']}, "
                f"{_hash_initializer(authority['identity']['path_hash'])}, "
                f"{authority['identity']['reset_epoch']}}}, "
                f"{_cpp_number(authority['s_commit'])}, "
                f"{authority['release_cycle_id']}, "
                f"{authority['release_generation']}, "
                f"{authority['history_generation']}}},"
            ),
            "  GoldenProjectionExpected{",
            f"    {_cpp_string(expected['status'])}, "
            + ", ".join(
                _cpp_number(expected[key])
                for key in (
                    "s",
                    "projected_x",
                    "projected_y",
                    "projected_heading",
                    "distance",
                    "signed_contour_error",
                    "heading_error",
                    "search_lower",
                    "search_upper",
                )
            )
            + ",",
            (
                f"    {expected['selected_segment']}, "
                f"{expected['accepted_candidate_count']}, "
                f"{expected['authority_cycle_id']}, "
                f"{expected['history_generation']}"
            ),
            "  }",
            "};",
            "",
        ]
    )
    return lines


def render_header(fixture: Mapping[str, Any]) -> str:
    _validate_integer_ranges(fixture)
    reference.validate_expected(fixture)
    lines = [
        "// Generated from test/fixtures/stage2_fg_execution_projection_golden_v1.json.",
        "// Build-tree-only artifact; do not install or include from production code.",
        "#pragma once",
        "",
        "#include <array>",
        "#include <cstddef>",
        "#include <cstdint>",
        "",
        "namespace spmpc_local_planner {",
        "namespace mainline {",
        "namespace stage2_fg_golden {",
        "",
        f"static constexpr char kSchemaVersion[] = {_cpp_string(fixture['schema_version'])};",
        f"static constexpr char kCanonicalJsonSha256[] = {_cpp_string(reference.canonical_sha256(fixture))};",
        f"static constexpr double kAbsoluteTolerance = {_cpp_number(fixture['numeric']['absolute_tolerance'])};",
        f"static constexpr std::size_t kKnownPrefixCaseCount = {len(fixture['known_prefix_cases'])};",
        f"static constexpr std::size_t kNominalProjectionCaseCount = {len(fixture['nominal_commit_cases'])};",
        "",
        "struct GoldenPose { double x; double y; double heading; };",
        "struct GoldenActual { double linear_velocity; double angular_velocity; };",
        "struct GoldenLiquid { double eta_x; double eta_x_dot; double eta_y; double eta_y_dot; };",
        "struct GoldenPhysical { GoldenPose pose; GoldenActual actual; GoldenLiquid liquid; };",
        "struct GoldenPublisher { double previous_linear_command; double previous_angular_command; double previous_linear_acceleration; double previous_angular_acceleration; };",
        "struct GoldenActuatorConfig { double dt_sec; double maximum_linear_delay_sec; double maximum_angular_delay_sec; double linear_delay_sec; double angular_delay_sec; double integer_snap_tolerance_ratio; double duration_tolerance_sec; };",
        "struct GoldenPlant { struct { double tau_sec; double gain; } linear_actuator; struct { double tau_sec; double gain; } angular_actuator; struct { double natural_frequency_rad_per_sec; double damping_ratio; double longitudinal_coupling; double lateral_coupling; } liquid; };",
        "struct GoldenPrefixEvent { std::uint64_t cycle_id; std::uint64_t release_generation; const char* emission_reason; double linear_command; double angular_command; double linear_acceleration; double angular_acceleration; std::uint64_t actual_lateness_ns; };",
        "struct GoldenTargetSegment { double duration_sec; double linear_target; double angular_target; };",
        "struct GoldenCoverage { const char* status; std::uint64_t history_generation; std::uint64_t predecessor_release_generation; std::size_t covered_event_count; std::int64_t maximum_adjacent_gap_ns; std::int64_t future_hold_ns; std::int64_t maximum_required_gap_ns; };",
        "",
        "struct GoldenProjectorConfig { double start_s_min; double start_s_max; double v_progress_bound; double forward_guard; double contour_guard; double heading_guard; double ambiguity_tolerance; double progress_equivalence_tolerance; double minimum_segment_length; };",
        "struct GoldenPathIdentity { std::uint64_t path_id; std::array<unsigned char, 32> path_hash; std::uint64_t reset_epoch; };",
        "struct GoldenPathVertex { double x; double y; double cumulative_s; };",
        "struct GoldenNominalAuthority { const char* kind; GoldenPathIdentity identity; double s_commit; std::uint64_t release_cycle_id; std::uint64_t release_generation; std::uint64_t history_generation; };",
        "struct GoldenProjectionExpected { const char* status; double s; double projected_x; double projected_y; double projected_heading; double distance; double signed_contour_error; double heading_error; double search_lower; double search_upper; std::size_t selected_segment; std::size_t accepted_candidate_count; std::uint64_t authority_cycle_id; std::uint64_t history_generation; };",
        "",
    ]
    for index, case in enumerate(fixture["known_prefix_cases"]):
        lines.extend(_known_case(index, case))
    for index, case in enumerate(fixture["nominal_commit_cases"]):
        lines.extend(_projection_case(index, case))
    lines.extend(
        [
            "}  // namespace stage2_fg_golden",
            "}  // namespace mainline",
            "}  // namespace spmpc_local_planner",
            "",
        ]
    )
    return "\n".join(lines)


def generate(fixture_path: Path, output_path: Path, *, check: bool = False) -> None:
    fixture = reference.load_fixture(fixture_path)
    rendered = render_header(fixture)
    if check:
        try:
            current = output_path.read_text(encoding="utf-8")
        except OSError as exc:
            raise reference.FixtureError(
                f"cannot read generated header for --check: {exc}"
            ) from exc
        if current != rendered:
            raise reference.FixtureError(
                "generated header is stale; regenerate it from the fixture"
            )
        return
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(rendered, encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    try:
        generate(args.fixture, args.output, check=args.check)
    except (reference.FixtureError, OSError) as exc:
        print(f"stage2 fg golden generation failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
