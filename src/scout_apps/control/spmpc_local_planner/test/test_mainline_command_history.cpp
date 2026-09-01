#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <thread>

#include "spmpc_local_planner/execution/command_event.h"
#include "spmpc_local_planner/execution/published_command_history.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr std::uint64_t kResetEpoch = 4;

ClockAnchor testAnchor() {
  return ClockAnchor{SteadyTimeNs(1000000000LL),
                     ModelTimeNs(9000000000LL)};
}

CycleRequest cycleAt(std::uint64_t cycle_id) {
  const ClockAnchor anchor = testAnchor();
  CycleRequest cycle;
  cycle.cycle_id = cycle_id;
  cycle.release_steady =
      ReleaseGridContract::boundary(anchor.steady, cycle_id);
  cycle.release_model = mapSteadyToModel(anchor, cycle.release_steady);
  cycle.reset_epoch = kResetEpoch;
  return cycle;
}

PlanarCommand command(double linear, double angular) {
  PlanarCommand result;
  result.linear = linear;
  result.angular = angular;
  return result;
}

PlanarCommandAcceleration acceleration(double linear, double angular) {
  PlanarCommandAcceleration result;
  result.linear = linear;
  result.angular = angular;
  return result;
}

EmittedCommandCommit makeCommit(
    std::uint64_t cycle_id, std::uint64_t expected_generation,
    const PlanarCommand& issued_command,
    const PlanarCommandAcceleration& authoritative_acceleration,
    EmissionReason reason = EmissionReason::kNominal,
    PublicationStatus publication_status = PublicationStatus::kPublished,
    std::int64_t steady_lateness_ns = 0,
    std::int64_t model_lateness_ns = 0) {
  EmittedCommandCommit result;
  result.cycle = cycleAt(cycle_id);
  result.expected_history_generation = expected_generation;
  result.receipt.status = publication_status;
  result.receipt.cycle = result.cycle;
  result.receipt.command = issued_command;
  result.receipt.actual_steady = SteadyTimeNs(
      result.cycle.release_steady.value + steady_lateness_ns);
  result.receipt.actual_model = ModelTimeNs(
      result.cycle.release_model.value + model_lateness_ns);
  result.command = issued_command;
  result.authoritative_acceleration = authoritative_acceleration;
  result.reason = reason;
  return result;
}

template <std::size_t Capacity>
bool snapshotIsSelfConsistent(
    const CommandHistorySnapshot<Capacity>& snapshot) {
  if (snapshot.empty()) {
    return snapshot.generation() == 0;
  }
  if (snapshot.size() > Capacity) {
    return false;
  }

  const PublishedCommandEvent& latest = snapshot.event(snapshot.size() - 1);
  if (latest.cycle.cycle_id < snapshot.size() - 1 ||
      latest.release_generation != latest.cycle.cycle_id + 1 ||
      snapshot.generation() != latest.release_generation ||
      snapshot.publisherState().previous_linear_command !=
          latest.publisher_state_after.previous_linear_command ||
      snapshot.publisherState().previous_angular_command !=
          latest.publisher_state_after.previous_angular_command ||
      snapshot.publisherState().previous_linear_acceleration !=
          latest.publisher_state_after.previous_linear_acceleration ||
      snapshot.publisherState().previous_angular_acceleration !=
          latest.publisher_state_after.previous_angular_acceleration) {
    return false;
  }

  const std::uint64_t first_cycle =
      latest.cycle.cycle_id - (snapshot.size() - 1);
  for (std::size_t index = 0; index < snapshot.size(); ++index) {
    const PublishedCommandEvent& event = snapshot.event(index);
    const std::uint64_t expected_cycle = first_cycle + index;
    const double expected_linear = static_cast<double>(expected_cycle);
    const double expected_angular = -expected_linear;
    const double expected_acceleration =
        static_cast<double>(expected_cycle % 7);
    if (event.cycle.cycle_id != expected_cycle ||
        event.cycle.reset_epoch != kResetEpoch ||
        event.release_generation != expected_cycle + 1 ||
        event.reason != EmissionReason::kNominal || event.publish_late ||
        event.command.linear != expected_linear ||
        event.command.angular != expected_angular ||
        event.publisher_state_after.previous_linear_command !=
            expected_linear ||
        event.publisher_state_after.previous_angular_command !=
            expected_angular ||
        event.publisher_state_after.previous_linear_acceleration !=
            expected_acceleration ||
        event.publisher_state_after.previous_angular_acceleration !=
            -expected_acceleration ||
        event.publisher_state_after.previous_linear_command !=
            event.command.linear ||
        event.publisher_state_after.previous_angular_command !=
            event.command.angular) {
      return false;
    }
  }
  return true;
}

template <std::size_t Capacity>
bool commitDeterministicSequence(PublishedCommandHistory<Capacity>& history,
                                 std::uint64_t count) {
  for (std::uint64_t cycle_id = 0; cycle_id < count; ++cycle_id) {
    const double value = static_cast<double>(cycle_id);
    const double accel = static_cast<double>(cycle_id % 7);
    const EmittedCommandCommit commit = makeCommit(
        cycle_id, cycle_id, command(value, -value),
        acceleration(accel, -accel));
    if (history.commitEmitted(commit) != HistoryCommitResult::kCommitted) {
      return false;
    }
  }
  return true;
}

TEST(MainlineCommandHistory, StartsEmpty) {
  PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
  CommandHistorySnapshot<4> snapshot;

  EXPECT_EQ(0u, history.generation());
  EXPECT_EQ(0u, history.nextCycleIdForWriter());
  EXPECT_FALSE(history.faultLatched());
  EXPECT_EQ(HistorySnapshotResult::kEmpty,
            history.capture(testAnchor().steady, testAnchor().model,
                            snapshot));
  EXPECT_TRUE(snapshot.empty());
}

TEST(MainlineCommandHistory, CommitsNominalCommandAuthorityAndGeneration) {
  PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
  const PlanarCommand issued = command(1.25, -0.5);
  const PlanarCommandAcceleration authoritative = acceleration(0.2, -0.3);
  PublishedCommandEvent committed;

  EXPECT_EQ(HistoryCommitResult::kCommitted,
            history.commitEmitted(
                makeCommit(0, 0, issued, authoritative), &committed));
  EXPECT_EQ(1u, history.generation());
  EXPECT_EQ(1u, history.nextCycleIdForWriter());
  EXPECT_EQ(0u, committed.cycle.cycle_id);
  EXPECT_EQ(1u, committed.release_generation);
  EXPECT_EQ(issued.linear, committed.command.linear);
  EXPECT_EQ(issued.angular, committed.command.angular);
  EXPECT_EQ(authoritative.linear,
            committed.publisher_state_after.previous_linear_acceleration);
  EXPECT_EQ(authoritative.angular,
            committed.publisher_state_after.previous_angular_acceleration);
  EXPECT_EQ(EmissionReason::kNominal, committed.reason);
  EXPECT_FALSE(committed.publish_late);

  CommandHistorySnapshot<4> snapshot;
  EXPECT_EQ(HistorySnapshotResult::kReady,
            history.capture(committed.actual_steady, committed.actual_model,
                            snapshot));
  ASSERT_EQ(1u, snapshot.size());
  EXPECT_EQ(1u, snapshot.generation());
  EXPECT_EQ(committed.cycle.cycle_id, snapshot.event(0).cycle.cycle_id);
  EXPECT_EQ(issued.linear,
            snapshot.publisherState().previous_linear_command);
  EXPECT_EQ(issued.angular,
            snapshot.publisherState().previous_angular_command);
  EXPECT_EQ(authoritative.linear,
            snapshot.publisherState().previous_linear_acceleration);
  EXPECT_EQ(authoritative.angular,
            snapshot.publisherState().previous_angular_acceleration);
}

TEST(MainlineCommandHistory,
     ResetsAccelerationForEveryNonNominalEmissionReason) {
  const std::array<EmissionReason, 6> reasons = {
      EmissionReason::kWarmupZero, EmissionReason::kDeadlineZero,
      EmissionReason::kSolverFailureZero, EmissionReason::kSafetyOverride,
      EmissionReason::kPublishJitterZero, EmissionReason::kClockFaultZero};

  for (const EmissionReason reason : reasons) {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    const bool safety_override = reason == EmissionReason::kSafetyOverride;
    const PlanarCommand issued =
        safety_override ? command(0.75, -0.25) : command(0.0, 0.0);
    const PlanarCommandAcceleration reset_acceleration = acceleration(0, 0);
    PublishedCommandEvent committed;

    ASSERT_TRUE(resetsPublisherAcceleration(reason));
    ASSERT_EQ(safety_override, !requiresZeroCommand(reason));
    ASSERT_EQ(HistoryCommitResult::kCommitted,
              history.commitEmitted(
                  makeCommit(0, 0, issued, reset_acceleration, reason),
                  &committed))
        << static_cast<int>(reason);
    EXPECT_EQ(reason, committed.reason);
    EXPECT_EQ(0.0,
              committed.publisher_state_after.previous_linear_acceleration);
    EXPECT_EQ(0.0,
              committed.publisher_state_after.previous_angular_acceleration);
    EXPECT_EQ(issued.linear, committed.command.linear);
    EXPECT_EQ(issued.angular, committed.command.angular);
  }
}

TEST(MainlineCommandHistory,
     WouldPublishAndFailedPublicationDoNotCreateHistoryFacts) {
  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    PublishedCommandEvent sentinel;
    sentinel.cycle.cycle_id = 99;
    EXPECT_EQ(HistoryCommitResult::kWouldPublish,
              history.commitEmitted(
                  makeCommit(0, 0, command(1, -1), acceleration(2, -2),
                             EmissionReason::kNominal,
                             PublicationStatus::kWouldPublish),
                  &sentinel));
    EXPECT_EQ(99u, sentinel.cycle.cycle_id);
    EXPECT_EQ(0u, history.generation());
    EXPECT_EQ(0u, history.nextCycleIdForWriter());
    EXPECT_FALSE(history.faultLatched());

    EXPECT_EQ(HistoryCommitResult::kCommitted,
              history.commitEmitted(makeCommit(
                  0, 0, command(1, -1), acceleration(2, -2))));
    EXPECT_EQ(1u, history.generation());
  }

  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    PublishedCommandEvent sentinel;
    sentinel.cycle.cycle_id = 99;
    EXPECT_EQ(HistoryCommitResult::kPublishFailed,
              history.commitEmitted(
                  makeCommit(0, 0, command(1, -1), acceleration(2, -2),
                             EmissionReason::kNominal,
                             PublicationStatus::kFailed),
                  &sentinel));
    EXPECT_EQ(99u, sentinel.cycle.cycle_id);
    EXPECT_EQ(0u, history.generation());
    EXPECT_EQ(0u, history.nextCycleIdForWriter());
    EXPECT_TRUE(history.faultLatched());

    CommandHistorySnapshot<4> snapshot;
    EXPECT_EQ(HistorySnapshotResult::kEmpty,
              history.capture(testAnchor().steady, testAnchor().model,
                              snapshot));
  }
}

TEST(MainlineCommandHistory, RejectsEpochCycleBoundaryAndGenerationMismatch) {
  const PlanarCommand issued = command(1, -1);
  const PlanarCommandAcceleration authoritative = acceleration(2, -2);
  const EmittedCommandCommit base =
      makeCommit(0, 0, issued, authoritative);

  const auto expectRejected = [&](const EmittedCommandCommit& invalid,
                                  HistoryCommitResult expected) {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    EXPECT_EQ(expected, history.commitEmitted(invalid));
    EXPECT_TRUE(history.faultLatched());
    EXPECT_EQ(0u, history.generation());
    EXPECT_EQ(0u, history.nextCycleIdForWriter());
    CommandHistorySnapshot<4> snapshot;
    EXPECT_EQ(HistorySnapshotResult::kEmpty,
              history.capture(testAnchor().steady, testAnchor().model,
                              snapshot));
  };

  EmittedCommandCommit wrong_epoch = base;
  wrong_epoch.cycle.reset_epoch = kResetEpoch + 1;
  expectRejected(wrong_epoch, HistoryCommitResult::kWrongEpoch);

  EmittedCommandCommit wrong_cycle = base;
  wrong_cycle.cycle = cycleAt(1);
  expectRejected(wrong_cycle, HistoryCommitResult::kWrongCycle);

  EmittedCommandCommit wrong_boundary = base;
  wrong_boundary.cycle.release_steady =
      SteadyTimeNs(wrong_boundary.cycle.release_steady.value + 1);
  expectRejected(wrong_boundary, HistoryCommitResult::kWrongBoundary);

  EmittedCommandCommit wrong_generation = base;
  wrong_generation.expected_history_generation = 1;
  expectRejected(wrong_generation,
                 HistoryCommitResult::kGenerationMismatch);

  EmittedCommandCommit unknown_reason = base;
  unknown_reason.reason = static_cast<EmissionReason>(255);
  expectRejected(unknown_reason, HistoryCommitResult::kInvalidReason);
}

TEST(MainlineCommandHistory, RejectsStaleReceiptAndReceiptMismatches) {
  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    ASSERT_EQ(HistoryCommitResult::kCommitted,
              history.commitEmitted(
                  makeCommit(0, 0, command(1, -1), acceleration(2, -2))));

    EmittedCommandCommit stale =
        makeCommit(1, 1, command(2, -2), acceleration(3, -3));
    stale.receipt.cycle = cycleAt(0);
    stale.receipt.command = command(1, -1);
    EXPECT_EQ(HistoryCommitResult::kReceiptMismatch,
              history.commitEmitted(stale));
    EXPECT_TRUE(history.faultLatched());
    EXPECT_EQ(1u, history.generation());
    EXPECT_EQ(1u, history.nextCycleIdForWriter());
  }

  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    EmittedCommandCommit wrong_command =
        makeCommit(0, 0, command(1, -1), acceleration(2, -2));
    wrong_command.receipt.command = command(1.5, -1);
    EXPECT_EQ(HistoryCommitResult::kReceiptMismatch,
              history.commitEmitted(wrong_command));
    EXPECT_TRUE(history.faultLatched());
  }

  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    EmittedCommandCommit wrong_cycle =
        makeCommit(0, 0, command(1, -1), acceleration(2, -2));
    wrong_cycle.receipt.cycle.cycle_id = 1;
    EXPECT_EQ(HistoryCommitResult::kReceiptMismatch,
              history.commitEmitted(wrong_cycle));
    EXPECT_TRUE(history.faultLatched());
  }
}

TEST(MainlineCommandHistory, RejectsNonFiniteCommandAndAuthority) {
  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    EmittedCommandCommit invalid =
        makeCommit(0, 0, command(1, -1), acceleration(2, -2));
    invalid.command.linear = std::numeric_limits<double>::quiet_NaN();
    invalid.receipt.command = invalid.command;
    EXPECT_EQ(HistoryCommitResult::kInvalidCommand,
              history.commitEmitted(invalid));
    EXPECT_TRUE(history.faultLatched());
  }

  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    EmittedCommandCommit invalid =
        makeCommit(0, 0, command(1, -1), acceleration(2, -2));
    invalid.authoritative_acceleration.angular =
        std::numeric_limits<double>::infinity();
    EXPECT_EQ(HistoryCommitResult::kInvalidAuthority,
              history.commitEmitted(invalid));
    EXPECT_TRUE(history.faultLatched());
  }
}

TEST(MainlineCommandHistory, RejectsInvalidSafetyResetValues) {
  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    EXPECT_EQ(HistoryCommitResult::kInvalidCommand,
              history.commitEmitted(makeCommit(
                  0, 0, command(0.01, 0.0), acceleration(0.0, 0.0),
                  EmissionReason::kDeadlineZero)));
    EXPECT_TRUE(history.faultLatched());
    EXPECT_EQ(0u, history.generation());
  }

  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    EXPECT_EQ(HistoryCommitResult::kInvalidAuthority,
              history.commitEmitted(makeCommit(
                  0, 0, command(0.2, -0.1), acceleration(0.01, 0.0),
                  EmissionReason::kSafetyOverride)));
    EXPECT_TRUE(history.faultLatched());
    EXPECT_EQ(0u, history.generation());
  }
}

TEST(MainlineCommandHistory, RejectsPublicationBeforeReleaseInBothClocks) {
  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    EXPECT_EQ(HistoryCommitResult::kPublishedTooEarly,
              history.commitEmitted(
                  makeCommit(0, 0, command(1, -1), acceleration(2, -2),
                             EmissionReason::kNominal,
                             PublicationStatus::kPublished, -1, 0)));
    EXPECT_TRUE(history.faultLatched());
  }

  {
    PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
    EXPECT_EQ(HistoryCommitResult::kPublishedTooEarly,
              history.commitEmitted(
                  makeCommit(0, 0, command(1, -1), acceleration(2, -2),
                             EmissionReason::kNominal,
                             PublicationStatus::kPublished, 0, -1)));
    EXPECT_TRUE(history.faultLatched());
  }
}

TEST(MainlineCommandHistory, RejectsActualClockRegression) {
  PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 200000000);
  ASSERT_EQ(HistoryCommitResult::kCommitted,
            history.commitEmitted(
                makeCommit(0, 0, command(1, -1), acceleration(2, -2),
                           EmissionReason::kNominal,
                           PublicationStatus::kPublished, 100000000,
                           100000000)));

  EXPECT_EQ(HistoryCommitResult::kActualClockRegression,
            history.commitEmitted(
                makeCommit(1, 1, command(2, -2), acceleration(3, -3),
                           EmissionReason::kNominal,
                           PublicationStatus::kPublished, 1, 1)));
  EXPECT_TRUE(history.faultLatched());
  EXPECT_EQ(1u, history.generation());
  EXPECT_EQ(1u, history.nextCycleIdForWriter());
}

TEST(MainlineCommandHistory, RejectsLateNominalPublication) {
  PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 5);
  EXPECT_EQ(HistoryCommitResult::kLateNominal,
            history.commitEmitted(
                makeCommit(0, 0, command(1, -1), acceleration(2, -2),
                           EmissionReason::kNominal,
                           PublicationStatus::kPublished, 6, 6)));
  EXPECT_TRUE(history.faultLatched());
  EXPECT_EQ(0u, history.generation());
  EXPECT_EQ(0u, history.nextCycleIdForWriter());
}

TEST(MainlineCommandHistory, RecordsLateSafetyZeroOnThePlannedModelGrid) {
  PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 5);
  PublishedCommandEvent committed;
  ASSERT_EQ(HistoryCommitResult::kCommitted,
            history.commitEmitted(
                makeCommit(0, 0, command(0, 0), acceleration(0, 0),
                           EmissionReason::kPublishJitterZero,
                           PublicationStatus::kPublished, 6, 9),
                &committed));
  EXPECT_TRUE(committed.publish_late);
  EXPECT_EQ(cycleAt(0).release_model.value,
            committed.cycle.release_model.value);
  EXPECT_EQ(cycleAt(0).release_model.value + 9,
            committed.actual_model.value);

  CommandHistorySnapshot<4> snapshot;
  ASSERT_EQ(HistorySnapshotResult::kReady,
            history.capture(committed.actual_steady, committed.actual_model,
                            snapshot));
  PublishedCommandEvent sampled;
  EXPECT_EQ(HistorySampleResult::kExact,
            snapshot.sampleAt(committed.cycle.release_model, sampled));
  EXPECT_EQ(HistorySampleResult::kFutureHold,
            snapshot.sampleAt(committed.actual_model, sampled));
  EXPECT_EQ(0u, sampled.cycle.cycle_id);
}

TEST(MainlineCommandHistory, RejectsInvalidCoverageAndSnapshotClockRegression) {
  CommandHistorySnapshot<4> empty_snapshot;
  EXPECT_EQ(HistoryCoverageStatus::kInvalidRange,
            empty_snapshot.coverage(ModelTimeNs(2), ModelTimeNs(1), 1)
                .status);
  EXPECT_EQ(HistoryCoverageStatus::kInvalidRange,
            empty_snapshot.coverage(ModelTimeNs(1), ModelTimeNs(2), -1)
                .status);

  PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
  const EmittedCommandCommit commit =
      makeCommit(0, 0, command(1, -1), acceleration(2, -2));
  ASSERT_EQ(HistoryCommitResult::kCommitted, history.commitEmitted(commit));
  CommandHistorySnapshot<4> snapshot;
  EXPECT_EQ(HistorySnapshotResult::kSnapshotClockRegression,
            history.capture(
                SteadyTimeNs(commit.receipt.actual_steady.value - 1),
                commit.receipt.actual_model, snapshot));
  EXPECT_TRUE(snapshot.empty());
}

TEST(MainlineCommandHistory, UsesAbsoluteThirtyHertzReleaseBoundaries) {
  PublishedCommandHistory<4> history(testAnchor(), kResetEpoch, 0);
  const std::array<std::uint64_t, 7> checkpoints = {0, 1, 2, 29, 30, 31,
                                                    300};
  std::size_t checkpoint_index = 0;

  for (std::uint64_t cycle_id = 0; cycle_id <= 300; ++cycle_id) {
    PublishedCommandEvent committed;
    const double value = static_cast<double>(cycle_id);
    ASSERT_EQ(HistoryCommitResult::kCommitted,
              history.commitEmitted(
                  makeCommit(cycle_id, cycle_id, command(value, -value),
                             acceleration(0, 0)),
                  &committed));
    if (cycle_id == checkpoints[checkpoint_index]) {
      const CycleRequest expected = cycleAt(cycle_id);
      EXPECT_EQ(expected.cycle_id, committed.cycle.cycle_id);
      EXPECT_EQ(expected.release_steady.value,
                committed.cycle.release_steady.value);
      EXPECT_EQ(expected.release_model.value,
                committed.cycle.release_model.value);
      ++checkpoint_index;
    }
  }
  EXPECT_EQ(checkpoints.size(), checkpoint_index);
  EXPECT_EQ(301u, history.generation());
}

TEST(MainlineCommandHistory, SamplesZohAndChecksCoveragePredecessorAndGap) {
  PublishedCommandHistory<8> history(testAnchor(), kResetEpoch, 0);
  ASSERT_TRUE(commitDeterministicSequence(history, 3));

  CommandHistorySnapshot<8> snapshot;
  ASSERT_EQ(HistorySnapshotResult::kReady,
            history.capture(SteadyTimeNs(5000000000LL),
                            ModelTimeNs(13000000000LL), snapshot));
  ASSERT_EQ(3u, snapshot.size());

  PublishedCommandEvent sampled;
  EXPECT_EQ(HistorySampleResult::kExact,
            snapshot.sampleAt(snapshot.event(1).cycle.release_model,
                              sampled));
  EXPECT_EQ(1u, sampled.cycle.cycle_id);

  const std::int64_t between =
      snapshot.event(0).cycle.release_model.value +
      (snapshot.event(1).cycle.release_model.value -
       snapshot.event(0).cycle.release_model.value) /
          2;
  EXPECT_EQ(HistorySampleResult::kHeldBetweenEvents,
            snapshot.sampleAt(ModelTimeNs(between), sampled));
  EXPECT_EQ(0u, sampled.cycle.cycle_id);

  EXPECT_EQ(HistorySampleResult::kFutureHold,
            snapshot.sampleAt(
                ModelTimeNs(snapshot.event(2).cycle.release_model.value + 1),
                sampled));
  EXPECT_EQ(2u, sampled.cycle.cycle_id);

  EXPECT_EQ(HistorySampleResult::kBeforeHistory,
            snapshot.sampleAt(
                ModelTimeNs(snapshot.event(0).cycle.release_model.value - 1),
                sampled));

  const std::int64_t maximum_30hz_gap = 33333334;
  const HistoryCoverage complete = snapshot.coverage(
      snapshot.event(0).cycle.release_model,
      snapshot.event(2).cycle.release_model, maximum_30hz_gap);
  EXPECT_EQ(HistoryCoverageStatus::kComplete, complete.status);
  EXPECT_EQ(1u, complete.predecessor_release_generation);
  EXPECT_EQ(3u, complete.covered_event_count);
  EXPECT_EQ(maximum_30hz_gap, complete.maximum_adjacent_gap_ns);
  EXPECT_EQ(0, complete.future_hold_ns);
  EXPECT_EQ(maximum_30hz_gap, complete.maximum_required_gap_ns);

  const HistoryCoverage missing_predecessor = snapshot.coverage(
      ModelTimeNs(snapshot.event(0).cycle.release_model.value - 1),
      snapshot.event(1).cycle.release_model, maximum_30hz_gap);
  EXPECT_EQ(HistoryCoverageStatus::kMissingPredecessor,
            missing_predecessor.status);

  const HistoryCoverage gap_too_large = snapshot.coverage(
      snapshot.event(0).cycle.release_model,
      snapshot.event(2).cycle.release_model, maximum_30hz_gap - 1);
  EXPECT_EQ(HistoryCoverageStatus::kGapTooLarge, gap_too_large.status);
}

TEST(MainlineCommandHistory, PrunesOldestEventsAtFixedCapacity) {
  PublishedCommandHistory<3> history(testAnchor(), kResetEpoch, 0);
  ASSERT_TRUE(commitDeterministicSequence(history, 5));

  CommandHistorySnapshot<3> snapshot;
  ASSERT_EQ(HistorySnapshotResult::kReady,
            history.capture(SteadyTimeNs(5000000000LL),
                            ModelTimeNs(13000000000LL), snapshot));
  ASSERT_EQ(3u, snapshot.size());
  EXPECT_EQ(5u, snapshot.generation());
  EXPECT_EQ(2u, snapshot.event(0).cycle.cycle_id);
  EXPECT_EQ(3u, snapshot.event(1).cycle.cycle_id);
  EXPECT_EQ(4u, snapshot.event(2).cycle.cycle_id);
  PublishedCommandEvent sampled;
  EXPECT_EQ(HistorySampleResult::kBeforeHistory,
            snapshot.sampleAt(ModelTimeNs(cycleAt(1).release_model.value),
                              sampled));
  EXPECT_TRUE(snapshotIsSelfConsistent(snapshot));
}

TEST(MainlineCommandHistory,
     SingleWriterAndConcurrentCaptureKeepSnapshotsConsistent) {
  constexpr std::uint64_t kCommitCount = 2000;
  PublishedCommandHistory<8> history(testAnchor(), kResetEpoch, 0);
  std::atomic<bool> start{false};
  std::atomic<bool> writer_done{false};
  std::atomic<bool> writer_failed{false};
  std::atomic<std::size_t> ready_snapshots{0};

  std::thread writer([&] {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    for (std::uint64_t cycle_id = 0; cycle_id < kCommitCount; ++cycle_id) {
      const double value = static_cast<double>(cycle_id);
      const double accel = static_cast<double>(cycle_id % 7);
      if (history.commitEmitted(makeCommit(
              cycle_id, cycle_id, command(value, -value),
              acceleration(accel, -accel))) !=
          HistoryCommitResult::kCommitted) {
        writer_failed.store(true, std::memory_order_release);
        break;
      }
      if ((cycle_id % 8) == 0) {
        std::this_thread::yield();
      }
    }
    writer_done.store(true, std::memory_order_release);
  });

  std::thread reader([&] {
    start.store(true, std::memory_order_release);
    for (std::size_t attempt = 0; attempt < 100000; ++attempt) {
      CommandHistorySnapshot<8> snapshot;
      const HistorySnapshotResult result = history.capture(
          SteadyTimeNs(100000000000LL), ModelTimeNs(108000000000LL),
          snapshot);
      if (result == HistorySnapshotResult::kReady) {
        if (!snapshotIsSelfConsistent(snapshot)) {
          writer_failed.store(true, std::memory_order_release);
          break;
        }
        ready_snapshots.fetch_add(1, std::memory_order_relaxed);
      } else if (result != HistorySnapshotResult::kEmpty &&
                 result != HistorySnapshotResult::kConcurrentWrite &&
                 result != HistorySnapshotResult::kReaderBusy &&
                 result != HistorySnapshotResult::kSnapshotClockRegression) {
        writer_failed.store(true, std::memory_order_release);
        break;
      }
      if (writer_done.load(std::memory_order_acquire) && attempt > 1000) {
        break;
      }
      std::this_thread::yield();
    }
  });

  writer.join();
  reader.join();

  EXPECT_FALSE(writer_failed.load(std::memory_order_acquire));
  EXPECT_FALSE(history.faultLatched());
  EXPECT_EQ(kCommitCount, history.generation());
  EXPECT_EQ(kCommitCount, history.nextCycleIdForWriter());
  EXPECT_GT(ready_snapshots.load(std::memory_order_acquire), 0u);

  CommandHistorySnapshot<8> final_snapshot;
  ASSERT_EQ(HistorySnapshotResult::kReady,
            history.capture(SteadyTimeNs(100000000000LL),
                            ModelTimeNs(108000000000LL), final_snapshot));
  ASSERT_EQ(8u, final_snapshot.size());
  EXPECT_EQ(kCommitCount, final_snapshot.generation());
  EXPECT_EQ(kCommitCount - 1,
            final_snapshot.event(final_snapshot.size() - 1)
                .cycle.cycle_id);
  EXPECT_TRUE(snapshotIsSelfConsistent(final_snapshot));
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
