#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <type_traits>

#include "spmpc_local_planner/execution/known_prefix_propagator.h"
#include "spmpc_local_planner/reference/reference_progress_projector.h"
#include "stage2_fg_execution_projection_golden_generated.hpp"

namespace spmpc_local_planner {
namespace mainline {
namespace {

namespace golden = stage2_fg_golden;

using PrefixCase = typename std::remove_cv<
    typename std::remove_reference<decltype(golden::kKnownPrefixCase0)>::type>::type;
using ProjectionCase = typename std::remove_cv<typename std::remove_reference<
    decltype(golden::kNominalProjectionCase0)>::type>::type;
using History = PublishedCommandHistory<PrefixCase::kHistoryCapacity>;
using Snapshot = CommandHistorySnapshot<PrefixCase::kHistoryCapacity>;
using Prefix = KnownPrefixPropagator<
    PrefixCase::kHistoryCapacity, PrefixCase::kLinearSelectorWidth,
    PrefixCase::kAngularSelectorWidth>;
using Projector =
    ReferenceProgressProjector<ProjectionCase::kVertexCapacity>;

static_assert(golden::kKnownPrefixCaseCount == 1,
              "Stage 2i v1 must contain one prefix case");
static_assert(golden::kNominalProjectionCaseCount == 1,
              "Stage 2i v1 must contain one projection case");

class Checker {
 public:
  void expect(bool condition, const std::string& field) {
    if (!condition) {
      ++failure_count_;
      std::cerr << "FAIL: " << field << '\n';
    }
  }

  void near(double expected, double actual, const std::string& field) {
    if (!std::isfinite(expected) || !std::isfinite(actual) ||
        std::fabs(expected - actual) > golden::kAbsoluteTolerance) {
      ++failure_count_;
      std::cerr << "FAIL: " << field << " expected=" << expected
                << " actual=" << actual << '\n';
    }
  }

  void text(const char* expected, const char* actual,
            const std::string& field) {
    expect(expected != nullptr && actual != nullptr &&
               std::strcmp(expected, actual) == 0,
           field);
  }

  int result() const {
    if (failure_count_ == 0) {
      std::cout << "Stage 2i emitted-history/prefix/projection golden: PASS\n";
      return 0;
    }
    std::cerr << failure_count_ << " Stage 2i golden checks failed\n";
    return 1;
  }

 private:
  std::size_t failure_count_{0};
};

CycleRequest cycleAt(const ClockAnchor& anchor, std::uint64_t reset_epoch,
                     std::uint64_t cycle_id) {
  CycleRequest cycle;
  cycle.cycle_id = cycle_id;
  cycle.release_steady = ReleaseGridContract::boundary(anchor.steady, cycle_id);
  cycle.release_model = mapSteadyToModel(anchor, cycle.release_steady);
  cycle.reset_epoch = reset_epoch;
  return cycle;
}

ActuatorDiscreteConfig actuatorConfig(
    const golden::GoldenActuatorConfig& source) {
  ActuatorDiscreteConfig result;
  result.dt_sec = source.dt_sec;
  result.maximum_linear_delay_sec = source.maximum_linear_delay_sec;
  result.maximum_angular_delay_sec = source.maximum_angular_delay_sec;
  result.linear_delay_sec = source.linear_delay_sec;
  result.angular_delay_sec = source.angular_delay_sec;
  result.integer_snap_tolerance_ratio = source.integer_snap_tolerance_ratio;
  result.duration_tolerance_sec = source.duration_tolerance_sec;
  return result;
}

ZohPlantParams plantParams(const golden::GoldenPlant& source) {
  ZohPlantParams result;
  result.linear_actuator.tau_sec = source.linear_actuator.tau_sec;
  result.linear_actuator.gain = source.linear_actuator.gain;
  result.angular_actuator.tau_sec = source.angular_actuator.tau_sec;
  result.angular_actuator.gain = source.angular_actuator.gain;
  result.liquid.natural_frequency_rad_per_sec =
      source.liquid.natural_frequency_rad_per_sec;
  result.liquid.damping_ratio = source.liquid.damping_ratio;
  result.liquid.longitudinal_coupling =
      source.liquid.longitudinal_coupling;
  result.liquid.lateral_coupling = source.liquid.lateral_coupling;
  return result;
}

PhysicalPlantState physicalState(const golden::GoldenPhysical& source) {
  PhysicalPlantState result;
  result.pose.x = source.pose.x;
  result.pose.y = source.pose.y;
  result.pose.heading = source.pose.heading;
  result.actual.linear_velocity = source.actual.linear_velocity;
  result.actual.angular_velocity = source.actual.angular_velocity;
  result.liquid.eta_x = source.liquid.eta_x;
  result.liquid.eta_x_dot = source.liquid.eta_x_dot;
  result.liquid.eta_y = source.liquid.eta_y;
  result.liquid.eta_y_dot = source.liquid.eta_y_dot;
  return result;
}

ReferencePathIdentity pathIdentity(
    const golden::GoldenPathIdentity& source) {
  ReferencePathIdentity result;
  result.path_id = source.path_id;
  for (std::size_t index = 0; index < result.path_hash.bytes.size(); ++index) {
    result.path_hash.bytes[index] =
        static_cast<std::uint8_t>(source.path_hash[index]);
  }
  result.reset_epoch = source.reset_epoch;
  return result;
}

ReferenceProgressProjectorConfig projectorConfig(
    const golden::GoldenProjectorConfig& source) {
  ReferenceProgressProjectorConfig result;
  result.start_s_min = source.start_s_min;
  result.start_s_max = source.start_s_max;
  result.v_progress_bound = source.v_progress_bound;
  result.forward_guard = source.forward_guard;
  result.contour_guard = source.contour_guard;
  result.heading_guard = source.heading_guard;
  result.ambiguity_tolerance = source.ambiguity_tolerance;
  result.progress_equivalence_tolerance =
      source.progress_equivalence_tolerance;
  result.minimum_segment_length = source.minimum_segment_length;
  return result;
}

void comparePhysical(const golden::GoldenPhysical& expected,
                     const PhysicalPlantState& actual, Checker& checker,
                     const std::string& prefix) {
  checker.near(expected.pose.x, actual.pose.x, prefix + ".pose.x");
  checker.near(expected.pose.y, actual.pose.y, prefix + ".pose.y");
  checker.near(expected.pose.heading, actual.pose.heading,
               prefix + ".pose.heading");
  checker.near(expected.actual.linear_velocity,
               actual.actual.linear_velocity,
               prefix + ".actual.linear_velocity");
  checker.near(expected.actual.angular_velocity,
               actual.actual.angular_velocity,
               prefix + ".actual.angular_velocity");
  checker.near(expected.liquid.eta_x, actual.liquid.eta_x,
               prefix + ".liquid.eta_x");
  checker.near(expected.liquid.eta_x_dot, actual.liquid.eta_x_dot,
               prefix + ".liquid.eta_x_dot");
  checker.near(expected.liquid.eta_y, actual.liquid.eta_y,
               prefix + ".liquid.eta_y");
  checker.near(expected.liquid.eta_y_dot, actual.liquid.eta_y_dot,
               prefix + ".liquid.eta_y_dot");
}

void commitFixtureHistory(const PrefixCase& fixture, const ClockAnchor& anchor,
                          History& history, Checker& checker) {
  for (const golden::GoldenPrefixEvent& source : fixture.history) {
    checker.text("nominal", source.emission_reason,
                 "prefix.history.emission_reason");
    EmittedCommandCommit commit;
    commit.cycle = cycleAt(anchor, fixture.reset_epoch, source.cycle_id);
    commit.expected_history_generation = source.release_generation - 1;
    commit.command = PlanarCommand{source.linear_command,
                                    source.angular_command};
    commit.authoritative_acceleration = PlanarCommandAcceleration{
        source.linear_acceleration, source.angular_acceleration};
    commit.reason = EmissionReason::kNominal;
    commit.receipt.status = PublicationStatus::kPublished;
    commit.receipt.cycle = commit.cycle;
    commit.receipt.command = commit.command;
    const std::int64_t actual_lateness_ns =
        static_cast<std::int64_t>(source.actual_lateness_ns);
    commit.receipt.actual_steady = SteadyTimeNs(
        commit.cycle.release_steady.value + actual_lateness_ns);
    commit.receipt.actual_model =
        ModelTimeNs(commit.cycle.release_model.value + actual_lateness_ns);

    PublishedCommandEvent committed;
    checker.expect(history.commitEmitted(commit, &committed) ==
                       HistoryCommitResult::kCommitted,
                   "prefix.history.commit");
    checker.expect(committed.release_generation == source.release_generation,
                   "prefix.history.release_generation");
  }
}

void comparePrefix(const PrefixCase& fixture, const Prefix::Result& actual,
                   Checker& checker) {
  const auto& expected = fixture.expected;
  checker.text("ok", expected.status, "prefix.expected.status");
  checker.expect(actual.segment_count == expected.segment_count,
                 "prefix.segment_count");
  const std::size_t comparable_segments =
      actual.segment_count < expected.segments.size()
          ? actual.segment_count
          : expected.segments.size();
  for (std::size_t index = 0; index < comparable_segments; ++index) {
    const std::string field =
        "prefix.segments[" + std::to_string(index) + "]";
    checker.near(expected.segments[index].duration_sec,
                 actual.segments[index].duration_sec,
                 field + ".duration_sec");
    checker.near(expected.segments[index].linear_target,
                 actual.segments[index].linear_target,
                 field + ".linear_target");
    checker.near(expected.segments[index].angular_target,
                 actual.segments[index].angular_target,
                 field + ".angular_target");
  }

  comparePhysical(expected.physical, actual.state.physical, checker,
                  "prefix.physical");
  checker.near(expected.publisher.previous_linear_command,
               actual.state.publisher.previous_linear_command,
               "prefix.publisher.previous_linear_command");
  checker.near(expected.publisher.previous_angular_command,
               actual.state.publisher.previous_angular_command,
               "prefix.publisher.previous_angular_command");
  checker.near(expected.publisher.previous_linear_acceleration,
               actual.state.publisher.previous_linear_acceleration,
               "prefix.publisher.previous_linear_acceleration");
  checker.near(expected.publisher.previous_angular_acceleration,
               actual.state.publisher.previous_angular_acceleration,
               "prefix.publisher.previous_angular_acceleration");
  for (std::size_t index = 0; index < actual.state.linear_older.size();
       ++index) {
    checker.near(expected.linear_older[index],
                 actual.state.linear_older[index],
                 "prefix.linear_older[" + std::to_string(index) + "]");
  }
  for (std::size_t index = 0; index < actual.state.angular_older.size();
       ++index) {
    checker.near(expected.angular_older[index],
                 actual.state.angular_older[index],
                 "prefix.angular_older[" + std::to_string(index) + "]");
  }

  checker.expect(actual.history_generation == expected.history_generation,
                 "prefix.history_generation");
  checker.expect(actual.last_emitted_cycle_id ==
                     expected.last_emitted_cycle_id,
                 "prefix.last_emitted_cycle_id");
  checker.expect(actual.start_model.value == expected.start_model_ns,
                 "prefix.start_model_ns");
  checker.expect(actual.target_model.value == expected.target_model_ns,
                 "prefix.target_model_ns");
  checker.text("complete", expected.coverage.status,
               "prefix.coverage.expected_status");
  checker.expect(actual.coverage.status == HistoryCoverageStatus::kComplete,
                 "prefix.coverage.status");
  checker.expect(actual.coverage.history_generation ==
                     expected.coverage.history_generation,
                 "prefix.coverage.history_generation");
  checker.expect(actual.coverage.predecessor_release_generation ==
                     expected.coverage.predecessor_release_generation,
                 "prefix.coverage.predecessor_release_generation");
  checker.expect(actual.coverage.covered_event_count ==
                     expected.coverage.covered_event_count,
                 "prefix.coverage.covered_event_count");
  checker.expect(actual.coverage.maximum_adjacent_gap_ns ==
                     expected.coverage.maximum_adjacent_gap_ns,
                 "prefix.coverage.maximum_adjacent_gap_ns");
  checker.expect(actual.coverage.future_hold_ns ==
                     expected.coverage.future_hold_ns,
                 "prefix.coverage.future_hold_ns");
  checker.expect(actual.coverage.maximum_required_gap_ns ==
                     expected.coverage.maximum_required_gap_ns,
                 "prefix.coverage.maximum_required_gap_ns");
}

void compareProjection(const ProjectionCase& fixture,
                       const ReferencePathIdentity& expected_identity,
                       const ReferenceProgressProjection& actual,
                       Checker& checker) {
  const golden::GoldenProjectionExpected& expected = fixture.expected;
  checker.text("ok", expected.status, "projection.expected.status");
  checker.near(expected.s, actual.s, "projection.s");
  checker.near(expected.projected_x, actual.projected_x,
               "projection.projected_x");
  checker.near(expected.projected_y, actual.projected_y,
               "projection.projected_y");
  checker.near(expected.projected_heading, actual.projected_heading,
               "projection.projected_heading");
  checker.near(expected.distance, actual.distance, "projection.distance");
  checker.near(expected.signed_contour_error,
               actual.signed_contour_error,
               "projection.signed_contour_error");
  checker.near(expected.heading_error, actual.heading_error,
               "projection.heading_error");
  checker.near(expected.search_lower, actual.search_lower,
               "projection.search_lower");
  checker.near(expected.search_upper, actual.search_upper,
               "projection.search_upper");
  checker.expect(actual.selected_segment == expected.selected_segment,
                 "projection.selected_segment");
  checker.expect(actual.accepted_candidate_count ==
                     expected.accepted_candidate_count,
                 "projection.accepted_candidate_count");
  checker.expect(actual.authority_cycle_id == expected.authority_cycle_id,
                 "projection.authority_cycle_id");
  checker.expect(actual.history_generation == expected.history_generation,
                 "projection.history_generation");
  checker.expect(actual.identity == expected_identity,
                 "projection.path_identity");
  checker.expect(actual.authority == ProgressAuthority::kNominalLiveRelease,
                 "projection.authority");
}

int runGolden() {
  Checker checker;
  checker.text("stage2_fg_execution_projection_golden_v1",
               golden::kSchemaVersion, "fixture.schema_version");
  checker.expect(std::strlen(golden::kCanonicalJsonSha256) == 64,
                 "fixture.canonical_sha256_length");

  const PrefixCase& prefix_fixture = golden::kKnownPrefixCase0;
  const ProjectionCase& projection_fixture = golden::kNominalProjectionCase0;
  checker.text(prefix_fixture.id,
               projection_fixture.source_known_prefix_case_id,
               "projection.source_known_prefix_case_id");

  const ClockAnchor anchor{SteadyTimeNs(prefix_fixture.anchor_steady_ns),
                           ModelTimeNs(prefix_fixture.anchor_model_ns)};
  History history(anchor, prefix_fixture.reset_epoch,
                  prefix_fixture.maximum_publish_lateness_ns,
                  prefix_fixture.history.front().cycle_id);
  commitFixtureHistory(prefix_fixture, anchor, history, checker);

  const CycleRequest latest_cycle =
      cycleAt(anchor, prefix_fixture.reset_epoch,
              prefix_fixture.history.back().cycle_id);
  Snapshot snapshot;
  checker.expect(
      history.capture(
          SteadyTimeNs(latest_cycle.release_steady.value +
                       prefix_fixture.snapshot_lateness_ns),
          ModelTimeNs(latest_cycle.release_model.value +
                      prefix_fixture.snapshot_lateness_ns),
          snapshot) == HistorySnapshotResult::kReady,
      "prefix.history.capture");
  const std::uint64_t generation_before = history.generation();

  const Prefix prefix(actuatorConfig(prefix_fixture.config),
                      plantParams(prefix_fixture.plant),
                      prefix_fixture.maximum_history_gap_ns);
  Prefix::Result prefix_result;
  const CycleRequest target_cycle = cycleAt(
      anchor, prefix_fixture.reset_epoch, prefix_fixture.target_cycle_id);
  checker.expect(
      prefix.propagate(physicalState(prefix_fixture.initial_physical),
                       ModelTimeNs(prefix_fixture.start_model_ns),
                       target_cycle, snapshot, prefix_result) ==
          KnownPrefixStatus::kOk,
      "prefix.propagate.status");
  comparePrefix(prefix_fixture, prefix_result, checker);
  checker.expect(history.generation() == generation_before,
                 "prefix.history_generation_unchanged");

  checker.text("nominal_live_release", projection_fixture.authority.kind,
               "projection.authority.kind");
  Projector::Path path;
  path.identity = pathIdentity(projection_fixture.identity);
  for (std::size_t index = 0; index < projection_fixture.vertices.size();
       ++index) {
    path.vertices[index].x = projection_fixture.vertices[index].x;
    path.vertices[index].y = projection_fixture.vertices[index].y;
    path.vertices[index].cumulative_s =
        projection_fixture.vertices[index].cumulative_s;
  }
  path.vertex_count = projection_fixture.vertices.size();
  path.s_path_end = projection_fixture.s_path_end;

  NominalProgressCommit authority;
  authority.source = ProgressAuthority::kNominalLiveRelease;
  authority.s_commit = projection_fixture.authority.s_commit;
  authority.identity = pathIdentity(projection_fixture.authority.identity);
  authority.release_cycle =
      cycleAt(anchor, prefix_fixture.reset_epoch,
              projection_fixture.authority.release_cycle_id);
  authority.release_generation =
      projection_fixture.authority.release_generation;
  authority.history_generation =
      projection_fixture.authority.history_generation;

  const Projector projector(projectorConfig(projection_fixture.config));
  ReferenceProgressProjection projection;
  checker.expect(
      projector.projectNominalCommit(
          prefix_result.state.physical.pose, target_cycle, path.identity,
          prefix_result.history_generation, authority, path, projection) ==
          ReferenceProgressStatus::kOk,
      "projection.status");
  compareProjection(projection_fixture, path.identity, projection, checker);
  return checker.result();
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner

int main() {
  try {
    return spmpc_local_planner::mainline::runGolden();
  } catch (const std::exception& error) {
    std::cerr << "Stage 2i golden threw: " << error.what() << '\n';
    return 2;
  }
}
