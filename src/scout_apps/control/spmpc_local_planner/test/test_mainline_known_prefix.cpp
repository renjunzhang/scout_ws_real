#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/execution/known_prefix_propagator.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr std::size_t kHistoryCapacity = 16;
constexpr std::uint64_t kResetEpoch = 7;
constexpr std::int64_t kMaximumGridGapNs = 33333334;
constexpr double kTolerance = 1e-12;

using History = PublishedCommandHistory<kHistoryCapacity>;
using Snapshot = CommandHistorySnapshot<kHistoryCapacity>;
using Prefix = KnownPrefixPropagator<kHistoryCapacity, 4, 4>;
using PrefixResult = Prefix::Result;

ClockAnchor testAnchor() {
  return ClockAnchor{SteadyTimeNs(1000000000LL), ModelTimeNs(0)};
}

CycleRequest cycleAt(std::uint64_t cycle_id,
                     std::uint64_t reset_epoch = kResetEpoch) {
  const ClockAnchor anchor = testAnchor();
  CycleRequest cycle;
  cycle.cycle_id = cycle_id;
  cycle.release_steady =
      ReleaseGridContract::boundary(anchor.steady, cycle_id);
  cycle.release_model = mapSteadyToModel(anchor, cycle.release_steady);
  cycle.reset_epoch = reset_epoch;
  return cycle;
}

ActuatorDiscreteConfig makeConfig() {
  ActuatorDiscreteConfig config;
  config.dt_sec = 1.0 / 30.0;
  config.maximum_linear_delay_sec = 3.0 * config.dt_sec;
  config.maximum_angular_delay_sec = 3.0 * config.dt_sec;
  config.linear_delay_sec = 0.5 * config.dt_sec;
  config.angular_delay_sec = 1.25 * config.dt_sec;
  config.integer_snap_tolerance_ratio = kTolerance;
  config.duration_tolerance_sec = kTolerance;
  return config;
}

ZohPlantParams makePlantParams() {
  ZohPlantParams params;
  params.linear_actuator = FopdtChannelParams{0.2, 1.0};
  params.angular_actuator = FopdtChannelParams{0.4, 1.0};
  params.liquid = LiquidModalParams{1.3, 0.2, 1.1, 0.9};
  return params;
}

PhysicalPlantState makePhysical() {
  PhysicalPlantState state;
  state.pose = PlanarPoseState{0.1, -0.2, 0.3};
  state.actual = ActualMotionState{0.2, -0.1};
  state.liquid = LiquidModalState{0.01, -0.02, 0.03, -0.04};
  return state;
}

EmittedCommandCommit makeCommit(std::uint64_t cycle_id,
                                std::int64_t actual_lateness_ns) {
  const CycleRequest cycle = cycleAt(cycle_id);
  const double value = static_cast<double>(cycle_id);
  EmittedCommandCommit commit;
  commit.cycle = cycle;
  commit.expected_history_generation = cycle_id;
  commit.command = PlanarCommand{value, -0.5 * value};
  commit.authoritative_acceleration =
      PlanarCommandAcceleration{0.1 * value, -0.2 * value};
  commit.reason = EmissionReason::kNominal;
  commit.receipt.status = PublicationStatus::kPublished;
  commit.receipt.cycle = cycle;
  commit.receipt.command = commit.command;
  commit.receipt.actual_steady =
      SteadyTimeNs(cycle.release_steady.value + actual_lateness_ns);
  commit.receipt.actual_model =
      ModelTimeNs(cycle.release_model.value + actual_lateness_ns);
  return commit;
}

void populateThrough(History& history, std::uint64_t last_cycle,
                     bool jittered) {
  for (std::uint64_t cycle_id = 0; cycle_id <= last_cycle; ++cycle_id) {
    const std::int64_t lateness =
        jittered ? static_cast<std::int64_t>(cycle_id % 4) * 100000 : 0;
    ASSERT_EQ(HistoryCommitResult::kCommitted,
              history.commitEmitted(makeCommit(cycle_id, lateness)));
  }
}

Snapshot captureAfter(const History& history, std::uint64_t last_cycle,
                      std::int64_t capture_lateness_ns) {
  Snapshot snapshot;
  const CycleRequest last = cycleAt(last_cycle);
  EXPECT_EQ(HistorySnapshotResult::kReady,
            history.capture(
                SteadyTimeNs(last.release_steady.value +
                             capture_lateness_ns),
                ModelTimeNs(last.release_model.value + capture_lateness_ns),
                snapshot));
  return snapshot;
}

PrefixResult makeSentinel() {
  PrefixResult result;
  result.state.physical.pose = PlanarPoseState{101.0, 102.0, 103.0};
  result.state.physical.actual = ActualMotionState{104.0, 105.0};
  result.state.physical.liquid =
      LiquidModalState{106.0, 107.0, 108.0, 109.0};
  result.state.publisher =
      AuthoritativePublisherState{110.0, 111.0, 112.0, 113.0};
  result.state.linear_older = {{114.0, 115.0}};
  result.state.angular_older = {{116.0, 117.0}};
  for (std::size_t index = 0; index < result.segments.size(); ++index) {
    const double value = static_cast<double>(index);
    result.segments[index] =
        ZohTargetSegment{120.0 + value, 220.0 + value, 320.0 + value};
  }
  result.segment_count = 9;
  result.history_generation = 401;
  result.last_emitted_cycle_id = 402;
  result.start_model = ModelTimeNs(403);
  result.target_model = ModelTimeNs(404);
  result.coverage.status = HistoryCoverageStatus::kGapTooLarge;
  result.coverage.history_generation = 405;
  result.coverage.predecessor_release_generation = 406;
  result.coverage.covered_event_count = 7;
  result.coverage.maximum_adjacent_gap_ns = 408;
  result.coverage.future_hold_ns = 409;
  result.coverage.maximum_required_gap_ns = 410;
  return result;
}

void expectPhysicalDoubleEq(const PhysicalPlantState& expected,
                            const PhysicalPlantState& actual) {
  EXPECT_DOUBLE_EQ(expected.pose.x, actual.pose.x);
  EXPECT_DOUBLE_EQ(expected.pose.y, actual.pose.y);
  EXPECT_DOUBLE_EQ(expected.pose.heading, actual.pose.heading);
  EXPECT_DOUBLE_EQ(expected.actual.linear_velocity,
                   actual.actual.linear_velocity);
  EXPECT_DOUBLE_EQ(expected.actual.angular_velocity,
                   actual.actual.angular_velocity);
  EXPECT_DOUBLE_EQ(expected.liquid.eta_x, actual.liquid.eta_x);
  EXPECT_DOUBLE_EQ(expected.liquid.eta_x_dot, actual.liquid.eta_x_dot);
  EXPECT_DOUBLE_EQ(expected.liquid.eta_y, actual.liquid.eta_y);
  EXPECT_DOUBLE_EQ(expected.liquid.eta_y_dot, actual.liquid.eta_y_dot);
}

void expectResultDoubleEq(const PrefixResult& expected,
                          const PrefixResult& actual) {
  expectPhysicalDoubleEq(expected.state.physical, actual.state.physical);
  EXPECT_DOUBLE_EQ(expected.state.publisher.previous_linear_command,
                   actual.state.publisher.previous_linear_command);
  EXPECT_DOUBLE_EQ(expected.state.publisher.previous_angular_command,
                   actual.state.publisher.previous_angular_command);
  EXPECT_DOUBLE_EQ(expected.state.publisher.previous_linear_acceleration,
                   actual.state.publisher.previous_linear_acceleration);
  EXPECT_DOUBLE_EQ(expected.state.publisher.previous_angular_acceleration,
                   actual.state.publisher.previous_angular_acceleration);
  EXPECT_EQ(expected.state.linear_older, actual.state.linear_older);
  EXPECT_EQ(expected.state.angular_older, actual.state.angular_older);
  EXPECT_EQ(expected.segment_count, actual.segment_count);
  for (std::size_t index = 0; index < expected.segments.size(); ++index) {
    EXPECT_DOUBLE_EQ(expected.segments[index].duration_sec,
                     actual.segments[index].duration_sec);
    EXPECT_DOUBLE_EQ(expected.segments[index].linear_target,
                     actual.segments[index].linear_target);
    EXPECT_DOUBLE_EQ(expected.segments[index].angular_target,
                     actual.segments[index].angular_target);
  }
  EXPECT_EQ(expected.history_generation, actual.history_generation);
  EXPECT_EQ(expected.last_emitted_cycle_id,
            actual.last_emitted_cycle_id);
  EXPECT_EQ(expected.start_model.value, actual.start_model.value);
  EXPECT_EQ(expected.target_model.value, actual.target_model.value);
  EXPECT_EQ(expected.coverage.status, actual.coverage.status);
  EXPECT_EQ(expected.coverage.history_generation,
            actual.coverage.history_generation);
  EXPECT_EQ(expected.coverage.predecessor_release_generation,
            actual.coverage.predecessor_release_generation);
  EXPECT_EQ(expected.coverage.covered_event_count,
            actual.coverage.covered_event_count);
  EXPECT_EQ(expected.coverage.maximum_adjacent_gap_ns,
            actual.coverage.maximum_adjacent_gap_ns);
  EXPECT_EQ(expected.coverage.future_hold_ns,
            actual.coverage.future_hold_ns);
  EXPECT_EQ(expected.coverage.maximum_required_gap_ns,
            actual.coverage.maximum_required_gap_ns);
}

TEST(MainlineKnownPrefix, ValidatesFrozenConfigurationAndWidths) {
  EXPECT_NO_THROW(Prefix(makeConfig(), makePlantParams(),
                         kMaximumGridGapNs));
  EXPECT_THROW(Prefix(makeConfig(), makePlantParams(), -1),
               std::invalid_argument);

  ActuatorDiscreteConfig invalid = makeConfig();
  invalid.dt_sec = 1.0;
  EXPECT_THROW(Prefix(invalid, makePlantParams(), kMaximumGridGapNs),
               std::invalid_argument);

  invalid = makeConfig();
  invalid.maximum_linear_delay_sec =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) * 1e-9;
  EXPECT_THROW(Prefix(invalid, makePlantParams(), kMaximumGridGapNs),
               std::overflow_error);

  ZohPlantParams invalid_plant = makePlantParams();
  invalid_plant.linear_actuator.tau_sec = 0.0;
  EXPECT_THROW(Prefix(makeConfig(), invalid_plant, kMaximumGridGapNs),
               std::invalid_argument);

  ActuatorDiscreteConfig mixed = makeConfig();
  mixed.maximum_linear_delay_sec = 0.0;
  mixed.linear_delay_sec = 0.0;
  EXPECT_NO_THROW((KnownPrefixPropagator<kHistoryCapacity, 1, 4>(
      mixed, makePlantParams(), kMaximumGridGapNs)));
  mixed.maximum_linear_delay_sec = mixed.dt_sec;
  mixed.linear_delay_sec = mixed.dt_sec;
  EXPECT_NO_THROW((KnownPrefixPropagator<kHistoryCapacity, 2, 4>(
      mixed, makePlantParams(), kMaximumGridGapNs)));
}

TEST(MainlineKnownPrefix, PropagatesWithIndependentSelectorWidths) {
  using MixedPrefix =
      KnownPrefixPropagator<kHistoryCapacity, 1, 4>;
  static_assert(MixedPrefix::State::kLinearOlderCount == 0,
                "NQ=1 must not invent linear older state");
  static_assert(MixedPrefix::State::kAngularOlderCount == 2,
                "NQ=4 must retain two angular older commands");

  ActuatorDiscreteConfig config = makeConfig();
  config.maximum_linear_delay_sec = 0.0;
  config.linear_delay_sec = 0.0;
  const MixedPrefix prefix(config, makePlantParams(),
                           kMaximumGridGapNs);
  History history(testAnchor(), kResetEpoch, 1000000);
  populateThrough(history, 7, false);
  const Snapshot snapshot = captureAfter(history, 7, 0);
  MixedPrefix::Result result;

  ASSERT_EQ(KnownPrefixStatus::kOk,
            prefix.propagate(makePhysical(), ModelTimeNs(176000000),
                             cycleAt(8), snapshot, result));
  EXPECT_EQ(0u, result.state.linear_older.size());
  EXPECT_EQ((std::array<double, 2>{{-3.0, -2.5}}),
            result.state.angular_older);
  EXPECT_DOUBLE_EQ(7.0,
                   result.state.publisher.previous_linear_command);
}

TEST(MainlineKnownPrefix, MatchesIndependentNonuniformPhysicalGolden) {
  History history(testAnchor(), kResetEpoch, 1000000);
  populateThrough(history, 7, false);
  const Snapshot snapshot = captureAfter(history, 7, 0);
  const std::uint64_t generation_before = history.generation();
  const Prefix prefix(makeConfig(), makePlantParams(), kMaximumGridGapNs);
  PrefixResult result;

  ASSERT_EQ(KnownPrefixStatus::kOk,
            prefix.propagate(makePhysical(), ModelTimeNs(176000000),
                             cycleAt(8), snapshot, result));
  ASSERT_EQ(6u, result.segment_count);
  const std::array<double, 6> expected_duration{{
      0.0073333336666666655,
      0.024999999666666668,
      0.0083333333333333384,
      0.024999999666666661,
      0.0083333333333333315,
      0.016666667333333343,
  }};
  const std::array<double, 6> expected_linear{{4, 5, 5, 6, 6, 7}};
  const std::array<double, 6> expected_angular{{-2, -2, -2.5, -2.5, -3,
                                                -3}};
  for (std::size_t index = 0; index < result.segment_count; ++index) {
    EXPECT_NEAR(expected_duration[index],
                result.segments[index].duration_sec, 1e-16);
    EXPECT_DOUBLE_EQ(expected_linear[index],
                     result.segments[index].linear_target);
    EXPECT_DOUBLE_EQ(expected_angular[index],
                     result.segments[index].angular_target);
  }

  EXPECT_NEAR(0.20311850974626217, result.state.physical.pose.x, 1e-14);
  EXPECT_NEAR(-0.16982577392279685, result.state.physical.pose.y, 1e-14);
  EXPECT_NEAR(0.27037537832359243,
              result.state.physical.pose.heading, 1e-14);
  EXPECT_NEAR(2.2269903814851522,
              result.state.physical.actual.linear_velocity, 1e-14);
  EXPECT_NEAR(-0.58348831328728301,
              result.state.physical.actual.angular_velocity, 1e-14);
  EXPECT_NEAR(-0.088275822166507673,
              result.state.physical.liquid.eta_x, 1e-14);
  EXPECT_NEAR(-2.1951067727774145,
              result.state.physical.liquid.eta_x_dot, 1e-14);
  EXPECT_NEAR(0.027209713334967387,
              result.state.physical.liquid.eta_y, 1e-14);
  EXPECT_NEAR(-0.0047302016378731101,
              result.state.physical.liquid.eta_y_dot, 1e-14);

  EXPECT_DOUBLE_EQ(7.0,
                   result.state.publisher.previous_linear_command);
  EXPECT_DOUBLE_EQ(-3.5,
                   result.state.publisher.previous_angular_command);
  EXPECT_DOUBLE_EQ(0.7,
                   result.state.publisher.previous_linear_acceleration);
  EXPECT_DOUBLE_EQ(-1.4,
                   result.state.publisher.previous_angular_acceleration);
  EXPECT_EQ((std::array<double, 2>{{6.0, 5.0}}),
            result.state.linear_older);
  EXPECT_EQ((std::array<double, 2>{{-3.0, -2.5}}),
            result.state.angular_older);
  EXPECT_EQ(8u, result.history_generation);
  EXPECT_EQ(7u, result.last_emitted_cycle_id);
  EXPECT_EQ(3u, result.coverage.predecessor_release_generation);
  EXPECT_EQ(6u, result.coverage.covered_event_count);
  EXPECT_EQ(kMaximumGridGapNs,
            result.coverage.maximum_adjacent_gap_ns);
  EXPECT_EQ(kMaximumGridGapNs, result.coverage.future_hold_ns);
  EXPECT_EQ(kMaximumGridGapNs,
            result.coverage.maximum_required_gap_ns);
  EXPECT_EQ(generation_before, history.generation());
}

TEST(MainlineKnownPrefix, PlannedTimelineIgnoresActualPublishJitter) {
  History exact_history(testAnchor(), kResetEpoch, 1000000);
  History jittered_history(testAnchor(), kResetEpoch, 1000000);
  populateThrough(exact_history, 7, false);
  populateThrough(jittered_history, 7, true);
  const Snapshot exact = captureAfter(exact_history, 7, 0);
  const Snapshot jittered = captureAfter(jittered_history, 7, 300000);
  const Prefix prefix(makeConfig(), makePlantParams(), kMaximumGridGapNs);
  PrefixResult exact_result;
  PrefixResult jittered_result;

  ASSERT_EQ(KnownPrefixStatus::kOk,
            prefix.propagate(makePhysical(), ModelTimeNs(176000000),
                             cycleAt(8), exact, exact_result));
  ASSERT_EQ(KnownPrefixStatus::kOk,
            prefix.propagate(makePhysical(), ModelTimeNs(176000000),
                             cycleAt(8), jittered, jittered_result));
  expectResultDoubleEq(exact_result, jittered_result);
}

TEST(MainlineKnownPrefix, UsesRightContinuousStartAndExclusiveTarget) {
  History history(testAnchor(), kResetEpoch, 1000000);
  populateThrough(history, 7, false);
  const Snapshot snapshot = captureAfter(history, 7, 0);
  ActuatorDiscreteConfig config = makeConfig();
  config.linear_delay_sec = 8333333.0e-9;
  config.angular_delay_sec = config.dt_sec;
  const Prefix prefix(config, makePlantParams(), kMaximumGridGapNs);
  PrefixResult result;

  ASSERT_EQ(KnownPrefixStatus::kOk,
            prefix.propagate(makePhysical(), ModelTimeNs(175000000),
                             cycleAt(8), snapshot, result));
  ASSERT_GT(result.segment_count, 0u);
  EXPECT_DOUBLE_EQ(5.0, result.segments[0].linear_target);
  EXPECT_DOUBLE_EQ(-3.0,
                   result.segments[result.segment_count - 1].angular_target);
  EXPECT_DOUBLE_EQ(-3.5,
                   result.state.publisher.previous_angular_command);
}

TEST(MainlineKnownPrefix, ZeroDurationIsPhysicalIdentityButRebuildsAuthority) {
  History history(testAnchor(), kResetEpoch, 1000000);
  populateThrough(history, 7, false);
  const std::int64_t to_target =
      cycleAt(8).release_model.value - cycleAt(7).release_model.value;
  const Snapshot snapshot = captureAfter(history, 7, to_target);
  const Prefix prefix(makeConfig(), makePlantParams(), kMaximumGridGapNs);
  PrefixResult result;
  const PhysicalPlantState initial = makePhysical();

  ASSERT_EQ(KnownPrefixStatus::kOk,
            prefix.propagate(initial, cycleAt(8).release_model, cycleAt(8),
                             snapshot, result));
  EXPECT_EQ(0u, result.segment_count);
  expectPhysicalDoubleEq(initial, result.state.physical);
  EXPECT_DOUBLE_EQ(7.0,
                   result.state.publisher.previous_linear_command);
  EXPECT_EQ((std::array<double, 2>{{6.0, 5.0}}),
            result.state.linear_older);
}

TEST(MainlineKnownPrefix, RejectsCoverageFailuresWithoutChangingOutput) {
  History history(testAnchor(), kResetEpoch, 1000000);
  populateThrough(history, 7, false);
  const Snapshot snapshot = captureAfter(history, 7, 0);
  const PrefixResult sentinel = makeSentinel();
  PrefixResult output = sentinel;

  const Prefix complete(makeConfig(), makePlantParams(),
                        kMaximumGridGapNs);
  EXPECT_EQ(KnownPrefixStatus::kMissingPredecessor,
            complete.propagate(makePhysical(), ModelTimeNs(50000000),
                               cycleAt(8), snapshot, output));
  expectResultDoubleEq(sentinel, output);

  const Prefix strict_gap(makeConfig(), makePlantParams(),
                          kMaximumGridGapNs - 1);
  EXPECT_EQ(KnownPrefixStatus::kHistoryGapTooLarge,
            strict_gap.propagate(makePhysical(), ModelTimeNs(176000000),
                                 cycleAt(8), snapshot, output));
  expectResultDoubleEq(sentinel, output);
}

TEST(MainlineKnownPrefix, RejectsCausalityAndEpochFailuresAtomically) {
  History history(testAnchor(), kResetEpoch, 1000000);
  populateThrough(history, 7, false);
  const Snapshot snapshot = captureAfter(history, 7, 0);
  const Prefix prefix(makeConfig(), makePlantParams(), kMaximumGridGapNs);
  const PrefixResult sentinel = makeSentinel();
  PrefixResult output = sentinel;

  Snapshot empty;
  EXPECT_EQ(KnownPrefixStatus::kEmptyHistory,
            prefix.propagate(makePhysical(), ModelTimeNs(0), cycleAt(8),
                             empty, output));
  expectResultDoubleEq(sentinel, output);

  EXPECT_EQ(KnownPrefixStatus::kInvalidTimeRange,
            prefix.propagate(makePhysical(),
                             ModelTimeNs(cycleAt(8).release_model.value + 1),
                             cycleAt(8), snapshot, output));
  expectResultDoubleEq(sentinel, output);

  EXPECT_EQ(KnownPrefixStatus::kWrongResetEpoch,
            prefix.propagate(makePhysical(), ModelTimeNs(176000000),
                             cycleAt(8, kResetEpoch + 1), snapshot, output));
  expectResultDoubleEq(sentinel, output);

  History future_history(testAnchor(), kResetEpoch, 1000000);
  populateThrough(future_history, 8, false);
  const Snapshot future = captureAfter(future_history, 8, 0);
  EXPECT_EQ(KnownPrefixStatus::kFutureEvent,
            prefix.propagate(makePhysical(), ModelTimeNs(176000000),
                             cycleAt(8), future, output));
  expectResultDoubleEq(sentinel, output);

  const Snapshot after_target = captureAfter(
      history, 7,
      cycleAt(8).release_model.value - cycleAt(7).release_model.value + 1);
  EXPECT_EQ(KnownPrefixStatus::kInvalidTimeRange,
            prefix.propagate(makePhysical(), ModelTimeNs(176000000),
                             cycleAt(8), after_target, output));
  expectResultDoubleEq(sentinel, output);
}

TEST(MainlineKnownPrefix, RejectsNumericalFailuresAtomically) {
  History history(testAnchor(), kResetEpoch, 1000000);
  populateThrough(history, 7, false);
  const Snapshot snapshot = captureAfter(history, 7, 0);
  const Prefix prefix(makeConfig(), makePlantParams(), kMaximumGridGapNs);
  const PrefixResult sentinel = makeSentinel();
  PrefixResult output = sentinel;

  PhysicalPlantState invalid = makePhysical();
  invalid.pose.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(KnownPrefixStatus::kInvalidInitialState,
            prefix.propagate(invalid, ModelTimeNs(176000000), cycleAt(8),
                             snapshot, output));
  expectResultDoubleEq(sentinel, output);

  invalid = makePhysical();
  invalid.actual.linear_velocity = std::numeric_limits<double>::max();
  invalid.actual.angular_velocity = std::numeric_limits<double>::max();
  EXPECT_EQ(KnownPrefixStatus::kPlantPropagationFailure,
            prefix.propagate(invalid, ModelTimeNs(176000000), cycleAt(8),
                             snapshot, output));
  expectResultDoubleEq(sentinel, output);
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
