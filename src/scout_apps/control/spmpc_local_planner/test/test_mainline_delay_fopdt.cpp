#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/execution/actuator_response_params.h"
#include "spmpc_local_planner/execution/discrete_delay_queue.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr double kDt = 1.0 / 30.0;
constexpr double kMaximumDelay = 3.0 * kDt;
constexpr double kTolerance = 1e-12;
using SyntheticQueue = DiscreteDelayQueue<4>;

SyntheticQueue makeSyntheticQueue() {
  SyntheticQueue queue(kDt, kMaximumDelay, kTolerance, kTolerance);
  queue.restore(30.0, std::array<double, 2>{{20.0, 10.0}});
  return queue;
}

TEST(MainlineDelayQueue, UsesExactNqAndOlderStateLayout) {
  static_assert(SyntheticQueue::kSelectorWidth == 4, "NQ must be R+1");
  static_assert(SyntheticQueue::kOlderCount == 2, "D must be R-1");

  SyntheticQueue queue = makeSyntheticQueue();
  const std::array<double, 4> taps = queue.taps(40.0);
  EXPECT_DOUBLE_EQ(40.0, taps[0]);
  EXPECT_DOUBLE_EQ(30.0, taps[1]);
  EXPECT_DOUBLE_EQ(20.0, taps[2]);
  EXPECT_DOUBLE_EQ(10.0, taps[3]);
}

TEST(MainlineDelayQueue, ShiftsOnlyTheFinalEmittedCommand) {
  SyntheticQueue queue = makeSyntheticQueue();
  queue.advanceAfterPublished(40.0);
  EXPECT_DOUBLE_EQ(40.0, queue.previousCommand());
  ASSERT_EQ(2u, queue.olderCommands().size());
  EXPECT_DOUBLE_EQ(30.0, queue.olderCommands()[0]);
  EXPECT_DOUBLE_EQ(20.0, queue.olderCommands()[1]);
  EXPECT_EQ((std::array<double, 4>{{50.0, 40.0, 30.0, 20.0}}),
            queue.taps(50.0));

  queue.advanceAfterPublished(50.0);
  EXPECT_EQ((std::array<double, 4>{{60.0, 50.0, 40.0, 30.0}}),
            queue.taps(60.0));
}

TEST(MainlineDelayQueue, ResetAndRestoreRejectNonFiniteState) {
  SyntheticQueue queue(kDt, kMaximumDelay);
  queue.resetToConstant(-2.0);
  EXPECT_EQ((std::array<double, 4>{{5.0, -2.0, -2.0, -2.0}}),
            queue.taps(5.0));
  EXPECT_THROW(
      queue.resetToConstant(std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
  EXPECT_THROW(queue.restore(
                   0.0, std::array<double, 2>{
                            {0.0, std::numeric_limits<double>::infinity()}}),
               std::invalid_argument);
  EXPECT_THROW(queue.advanceAfterPublished(
                   std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_THROW(queue.taps(std::numeric_limits<double>::infinity()),
               std::invalid_argument);
}

TEST(MainlineDelayQueue, RequiresAuthoritativeHistoryOnlyWhenTapsNeedIt) {
  DiscreteDelayQueue<2> one_step(kDt, kDt);
  EXPECT_FALSE(one_step.initialized());
  EXPECT_THROW(one_step.taps(1.0), std::logic_error);
  double target = 123.0;
  EXPECT_FALSE(one_step.select(1.0, 0, target));
  EXPECT_DOUBLE_EQ(123.0, target);
  EXPECT_FALSE(one_step.tryAdvanceAfterPublished(1.0));
  EXPECT_THROW(one_step.advanceAfterPublished(1.0), std::logic_error);

  one_step.restore(0.0, std::array<double, 0>{});
  EXPECT_TRUE(one_step.initialized());
  one_step.clear();
  EXPECT_FALSE(one_step.initialized());
}

TEST(MainlineDelayQueue, SelectsAllIntegerDelayBoundariesIncludingLmax) {
  const SyntheticQueue queue = makeSyntheticQueue();
  const std::array<double, 4> taps = queue.taps(40.0);
  for (std::size_t integer_steps = 0; integer_steps <= 3;
       ++integer_steps) {
    const ChannelDelaySchedule<4> schedule =
        queue.schedule(static_cast<double>(integer_steps) * kDt);
    ASSERT_TRUE(schedule.valid(kDt, kTolerance));
    EXPECT_EQ(integer_steps, schedule.integer_delay_steps);
    EXPECT_DOUBLE_EQ(0.0, schedule.fractional_beta);
    EXPECT_DOUBLE_EQ(0.0, schedule.duration[0]);
    EXPECT_DOUBLE_EQ(kDt, schedule.duration[1]);
    EXPECT_DOUBLE_EQ(0.0, schedule.duration[2]);
    EXPECT_EQ(0u, delaySelectorIndex(schedule.selector[0]));
    EXPECT_EQ(integer_steps, delaySelectorIndex(schedule.selector[1]));
    EXPECT_EQ(0u, delaySelectorIndex(schedule.selector[2]));
    EXPECT_DOUBLE_EQ(taps[integer_steps],
                     selectDelayTarget(taps, schedule.selector[1]));
  }
}

TEST(MainlineDelayQueue, SelectsOldThenNewTargetsForFractionalDelay) {
  const SyntheticQueue queue = makeSyntheticQueue();
  const std::array<double, 4> taps = queue.taps(40.0);
  for (std::size_t integer_steps = 0; integer_steps <= 2;
       ++integer_steps) {
    const ChannelDelaySchedule<4> schedule = queue.schedule(
        (static_cast<double>(integer_steps) + 0.5) * kDt);
    ASSERT_TRUE(schedule.valid(kDt, kTolerance));
    EXPECT_EQ(integer_steps, schedule.integer_delay_steps);
    EXPECT_DOUBLE_EQ(0.5, schedule.fractional_beta);
    EXPECT_NEAR(0.5 * kDt, schedule.duration[0], 1e-15);
    EXPECT_NEAR(0.5 * kDt, schedule.duration[1], 1e-15);
    EXPECT_EQ(integer_steps + 1,
              delaySelectorIndex(schedule.selector[0]));
    EXPECT_EQ(integer_steps, delaySelectorIndex(schedule.selector[1]));
    EXPECT_DOUBLE_EQ(taps[integer_steps + 1],
                     selectDelayTarget(taps, schedule.selector[0]));
    EXPECT_DOUBLE_EQ(taps[integer_steps],
                     selectDelayTarget(taps, schedule.selector[1]));
  }
}

TEST(MainlineDelayQueue, SnapsBothSidesOfAnIntegerRatio) {
  const SyntheticQueue queue = makeSyntheticQueue();
  const ChannelDelaySchedule<4> below =
      queue.schedule(kDt * (1.0 - 0.5 * kTolerance));
  const ChannelDelaySchedule<4> above =
      queue.schedule(kDt * (1.0 + 0.5 * kTolerance));
  for (const ChannelDelaySchedule<4>* schedule : {&below, &above}) {
    EXPECT_EQ(1u, schedule->integer_delay_steps);
    EXPECT_DOUBLE_EQ(0.0, schedule->fractional_beta);
    EXPECT_DOUBLE_EQ(0.0, schedule->duration[0]);
    EXPECT_EQ(1u, delaySelectorIndex(schedule->selector[1]));
  }
}

TEST(MainlineDelayQueue, SupportsRZeroROneAndNonintegerMaximum) {
  static_assert(DiscreteDelayQueue<1>::kOlderCount == 0,
                "R=0 has no delay history state");
  static_assert(DiscreteDelayQueue<2>::kOlderCount == 0,
                "R=1 uses q_prev but no older state");

  DiscreteDelayQueue<1> no_delay(kDt, 0.0);
  EXPECT_TRUE(no_delay.initialized());
  EXPECT_EQ((std::array<double, 1>{{4.0}}), no_delay.taps(4.0));
  double selected = -1.0;
  EXPECT_TRUE(no_delay.select(4.0, 0, selected));
  EXPECT_DOUBLE_EQ(4.0, selected);
  no_delay.advanceAfterPublished(4.0);
  EXPECT_EQ(0u,
            delaySelectorIndex(no_delay.schedule(0.0).selector[1]));

  DiscreteDelayQueue<2> one_step(kDt, kDt);
  one_step.restore(3.0, std::array<double, 0>{});
  EXPECT_EQ((std::array<double, 2>{{4.0, 3.0}}), one_step.taps(4.0));
  EXPECT_EQ(1u,
            delaySelectorIndex(one_step.schedule(kDt).selector[1]));

  DiscreteDelayQueue<2> half_step(kDt, 0.5 * kDt);
  const ChannelDelaySchedule<2> half = half_step.schedule(0.5 * kDt);
  EXPECT_EQ(1u, delaySelectorIndex(half.selector[0]));
  EXPECT_EQ(0u, delaySelectorIndex(half.selector[1]));
}

TEST(MainlineDelayQueue, RejectsInvalidConfigurationDelayAndWidth) {
  EXPECT_THROW((DiscreteDelayQueue<3>(kDt, kMaximumDelay)),
               std::invalid_argument);
  EXPECT_THROW((DiscreteDelayQueue<4>(0.0, kMaximumDelay)),
               std::invalid_argument);
  EXPECT_THROW((DiscreteDelayQueue<4>(kDt, -1.0)),
               std::invalid_argument);
  EXPECT_THROW((DiscreteDelayQueue<4>(kDt, kMaximumDelay, 0.5)),
               std::invalid_argument);
  EXPECT_THROW((DiscreteDelayQueue<4>(kDt, kMaximumDelay, kTolerance,
                                      kDt)),
               std::invalid_argument);

  const SyntheticQueue queue = makeSyntheticQueue();
  EXPECT_THROW(queue.schedule(-1.0), std::invalid_argument);
  EXPECT_THROW(queue.schedule(kMaximumDelay + 1e-9),
               std::invalid_argument);
  EXPECT_THROW(queue.schedule(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
}

TEST(MainlineDelayQueue, MergesIndependentChannelSwitchesIntoThreeSlots) {
  const SyntheticQueue linear_queue = makeSyntheticQueue();
  SyntheticQueue angular_queue(kDt, kMaximumDelay);
  angular_queue.restore(3.0, std::array<double, 2>{{2.0, 1.0}});
  const CombinedDelaySchedule<4, 4> merged = mergeDelaySchedules(
      linear_queue.schedule(0.5 * kDt),
      angular_queue.schedule(1.25 * kDt), kDt);
  ASSERT_TRUE(merged.valid(kDt, kTolerance));
  EXPECT_NEAR(0.25 * kDt, merged.duration[0], 1e-15);
  EXPECT_NEAR(0.25 * kDt, merged.duration[1], 1e-15);
  EXPECT_NEAR(0.50 * kDt, merged.duration[2], 1e-15);

  const std::array<double, 4> linear_taps = linear_queue.taps(40.0);
  const std::array<double, 4> angular_taps = angular_queue.taps(4.0);
  const std::array<double, 3> expected_linear{{30.0, 30.0, 40.0}};
  const std::array<double, 3> expected_angular{{2.0, 3.0, 3.0}};
  for (std::size_t slot = 0; slot < 3; ++slot) {
    EXPECT_DOUBLE_EQ(
        expected_linear[slot],
        selectDelayTarget(linear_taps, merged.linear_selector[slot]));
    EXPECT_DOUBLE_EQ(
        expected_angular[slot],
        selectDelayTarget(angular_taps, merged.angular_selector[slot]));
  }
}

TEST(MainlineDelayQueue, MergesDifferentWidthsAndPreservesTinySegments) {
  DiscreteDelayQueue<2> linear(kDt, kDt);
  SyntheticQueue angular(kDt, kMaximumDelay);
  const CombinedDelaySchedule<2, 4> mixed = mergeDelaySchedules(
      linear.schedule(0.5 * kDt), angular.schedule(1.25 * kDt), kDt);
  EXPECT_TRUE(mixed.valid(kDt, kTolerance));

  const CombinedDelaySchedule<4, 4> nearly_equal = mergeDelaySchedules(
      angular.schedule(0.5 * kDt),
      angular.schedule((0.5 + 2e-11) * kDt), kDt);
  EXPECT_TRUE(nearly_equal.valid(kDt, kTolerance));
  EXPECT_GT(nearly_equal.duration[0], 0.0);
  EXPECT_GT(nearly_equal.duration[1], 0.0);
  EXPECT_LT(nearly_equal.duration[1], kTolerance);
  EXPECT_GT(nearly_equal.duration[2], 0.0);
}

TEST(MainlineDelayQueue, RejectsMalformedSelectorsAndSchedules) {
  ChannelDelaySchedule<4> malformed = makeSyntheticQueue().schedule(0.5 * kDt);
  malformed.selector[0].fill(0.0);
  EXPECT_FALSE(malformed.valid(kDt, kTolerance));
  malformed = makeSyntheticQueue().schedule(0.5 * kDt);
  malformed.duration[1] = -1.0;
  EXPECT_FALSE(malformed.valid(kDt, kTolerance));
  malformed = makeSyntheticQueue().schedule(0.5 * kDt);
  malformed.duration[0] = 0.0;
  malformed.duration[1] = kDt;
  EXPECT_FALSE(malformed.valid(kDt, kTolerance));

  std::array<double, 4> taps{{40.0, 30.0, 20.0, 10.0}};
  std::array<double, 4> selector{{1.0, 1.0, 0.0, 0.0}};
  double selected_target = 123.0;
  EXPECT_FALSE(trySelectDelayTarget(taps, selector, selected_target));
  EXPECT_DOUBLE_EQ(123.0, selected_target);
  EXPECT_THROW(selectDelayTarget(taps, selector), std::invalid_argument);
  selector = {{1.0, 0.0, 0.0, 0.0}};
  taps[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(trySelectDelayTarget(taps, selector, selected_target));
  EXPECT_DOUBLE_EQ(123.0, selected_target);
  EXPECT_THROW(selectDelayTarget(taps, selector), std::invalid_argument);

  const SyntheticQueue queue = makeSyntheticQueue();
  CombinedDelaySchedule<4, 4> combined = mergeDelaySchedules(
      queue.schedule(0.0), queue.schedule(0.0), kDt);
  combined.linear_selector[1] = {{0.0, 1.0, 0.0, 0.0}};
  EXPECT_FALSE(combined.valid(kDt, kTolerance));

  combined = mergeDelaySchedules(
      queue.schedule(0.0), queue.schedule(0.0), kDt);
  combined.duration = {{0.5 * kDt, 0.0, 0.5 * kDt}};
  EXPECT_FALSE(combined.valid(kDt, kTolerance));
}

TEST(MainlineDelayQueue, BuilderReturnsStableStatusAndLeavesOutputUntouched) {
  CombinedDelaySchedule<4, 4> output;
  for (std::size_t slot = 0; slot < kActuatorExecutionSegmentCount; ++slot) {
    output.duration[slot] = 100.0 + static_cast<double>(slot);
    output.linear_selector[slot].fill(0.0);
    output.angular_selector[slot].fill(0.0);
    output.linear_selector[slot][3] = 1.0;
    output.angular_selector[slot][2] = 1.0;
  }
  const CombinedDelaySchedule<4, 4> sentinel = output;

  const auto expectFailure = [&](double dt, double max_linear,
                                 double max_angular, double delay_linear,
                                 double delay_angular,
                                 double snap_tolerance,
                                 DelayScheduleStatus expected) {
    EXPECT_EQ(expected,
              makeFractionalDelaySchedule(
                  dt, max_linear, max_angular, delay_linear, delay_angular,
                  snap_tolerance, output, kTolerance));
    EXPECT_EQ(sentinel.duration, output.duration);
    EXPECT_EQ(sentinel.linear_selector, output.linear_selector);
    EXPECT_EQ(sentinel.angular_selector, output.angular_selector);
  };

  expectFailure(0.0, kMaximumDelay, kMaximumDelay, 0.0, 0.0, 0.0,
                DelayScheduleStatus::kInvalidDt);
  expectFailure(kDt, kMaximumDelay, kMaximumDelay, -1.0, 0.0, 0.0,
                DelayScheduleStatus::kInvalidDelay);
  expectFailure(kDt, kMaximumDelay, kMaximumDelay, 0.0,
                std::numeric_limits<double>::infinity(), 0.0,
                DelayScheduleStatus::kInvalidDelay);
  expectFailure(kDt, kMaximumDelay, kMaximumDelay, kMaximumDelay + 1e-9,
                0.0, 0.0, DelayScheduleStatus::kDelayOutOfRange);
  expectFailure(kDt, kMaximumDelay, kMaximumDelay, 0.0, 0.0, -1.0,
                DelayScheduleStatus::kInvalidSnapTolerance);
  expectFailure(kDt, kMaximumDelay, kMaximumDelay, 0.0, 0.0, 0.5 * kDt,
                DelayScheduleStatus::kInvalidSnapTolerance);

  CombinedDelaySchedule<4, 4> built;
  EXPECT_EQ(DelayScheduleStatus::kOk,
            makeFractionalDelaySchedule(
                kDt, kMaximumDelay, kMaximumDelay, 0.5 * kDt, 1.25 * kDt,
                kTolerance, built, kTolerance));
  ASSERT_TRUE(built.valid(kDt, kTolerance));
  EXPECT_NEAR(0.25 * kDt, built.duration[0], 1e-15);
  EXPECT_NEAR(0.25 * kDt, built.duration[1], 1e-15);
  EXPECT_NEAR(0.50 * kDt, built.duration[2], 1e-15);

  CombinedDelaySchedule<1, 2> independent_widths;
  EXPECT_EQ(DelayScheduleStatus::kOk,
            makeFractionalDelaySchedule(
                kDt, 0.0, kDt, 0.0, 0.5 * kDt,
                kTolerance * kDt, independent_widths, kTolerance));
  ASSERT_TRUE(independent_widths.valid(kDt, kTolerance));
  EXPECT_EQ(0u,
            delaySelectorIndex(independent_widths.linear_selector[0]));
  EXPECT_EQ(1u,
            delaySelectorIndex(independent_widths.angular_selector[0]));
  EXPECT_EQ(0u,
            delaySelectorIndex(independent_widths.angular_selector[1]));

  CombinedDelaySchedule<2, 4> wrong_linear_width;
  wrong_linear_width.duration[0] = 321.0;
  EXPECT_EQ(DelayScheduleStatus::kSelectorOverflow,
            makeFractionalDelaySchedule(
                kDt, kMaximumDelay, kMaximumDelay, 0.0, 0.0, kTolerance,
                wrong_linear_width, kTolerance));
  EXPECT_DOUBLE_EQ(321.0, wrong_linear_width.duration[0]);

  EXPECT_EQ(DelayScheduleStatus::kInvalidSnapTolerance,
            makeFractionalDelaySchedule(
                kDt, kMaximumDelay, kMaximumDelay, 0.0, 0.0, 0.0,
                output, kDt));
  EXPECT_EQ(sentinel.duration, output.duration);
  EXPECT_EQ(sentinel.linear_selector, output.linear_selector);
  EXPECT_EQ(sentinel.angular_selector, output.angular_selector);
}

TEST(MainlineFopdt, MatchesZeroStepConstantAndReversalGoldenVectors) {
  const double log_two = std::log(2.0);
  EXPECT_DOUBLE_EQ(0.0,
                   stepFopdt(0.0, 0.0, 0.7,
                             ActuatorResponseParams{0.0, 0.1, 1.0}));
  EXPECT_NEAR(3.5,
              stepFopdt(1.0, 3.0, 0.1 * log_two,
                        ActuatorResponseParams{0.0, 0.1, 2.0}),
              1e-14);
  EXPECT_NEAR(3.0,
              stepFopdt(3.0, 2.0, 0.4,
                        ActuatorResponseParams{0.0, 0.1, 1.5}),
              1e-14);
  EXPECT_NEAR(0.0,
              stepFopdt(2.0, -2.0, 0.2 * log_two,
                        ActuatorResponseParams{0.0, 0.2, 1.0}),
              1e-14);
}

TEST(MainlineFopdt, PreservesZeroDurationAndShortStepPrecision) {
  const ActuatorResponseParams params{0.0, 1.0, 1.0};
  EXPECT_DOUBLE_EQ(0.7, stepFopdt(0.7, -2.0, 0.0, params));
  const double tiny = 1e-16;
  EXPECT_NEAR(tiny, stepFopdt(0.0, 1.0, tiny, params), 1e-30);
}

TEST(MainlineFopdt, SeparatesTauGainAndDelayRoles) {
  const double fast = stepFopdt(
      0.0, 1.0, 0.1, ActuatorResponseParams{0.0, 0.1, 1.0});
  const double slow = stepFopdt(
      0.0, 1.0, 0.1, ActuatorResponseParams{0.0, 0.2, 1.0});
  EXPECT_NEAR(0.6321205588285577, fast, 1e-15);
  EXPECT_NEAR(0.3934693402873666, slow, 1e-15);
  EXPECT_GT(fast, slow);

  const double log_two = std::log(2.0);
  EXPECT_NEAR(1.5,
              stepFopdt(0.0, 2.0, 0.1 * log_two,
                        ActuatorResponseParams{0.0, 0.1, 1.5}),
              1e-14);
  EXPECT_NEAR(0.5,
              stepFopdt(0.0, 2.0, 0.1 * log_two,
                        ActuatorResponseParams{0.0, 0.1, 0.5}),
              1e-14);
  EXPECT_DOUBLE_EQ(
      stepFopdt(0.2, 0.8, 0.03,
                ActuatorResponseParams{0.0, 0.1, 1.2}),
      stepFopdt(0.2, 0.8, 0.03,
                ActuatorResponseParams{9.0, 0.1, 1.2}));
  EXPECT_NEAR(50.0,
              fopdtAcceleration(
                  1.0, 3.0, ActuatorResponseParams{0.0, 0.1, 2.0}),
              1e-14);
}

TEST(MainlineFopdt, ComposesConstantAndPiecewiseTargetsInTimeOrder) {
  const ActuatorResponseParams params{0.0, 0.2, 1.0};
  const double split = stepFopdt(
      stepFopdt(0.3, -0.6, 0.03, params), -0.6, 0.07, params);
  const double whole = stepFopdt(0.3, -0.6, 0.10, params);
  EXPECT_NEAR(whole, split, 1e-15);

  const double old_then_new = stepFopdt(
      stepFopdt(0.0, -1.0, 0.05, params), 1.0, 0.05, params);
  const double averaged = stepFopdt(0.0, 0.0, 0.10, params);
  EXPECT_GT(old_then_new, averaged);
}

TEST(MainlineFopdt, RejectsInvalidParamsInputsAndNonFiniteOutputs) {
  EXPECT_THROW(validateActuatorResponseParams(
                   ActuatorResponseParams{-1.0, 0.1, 1.0}),
               std::invalid_argument);
  EXPECT_THROW(validateActuatorResponseParams(
                   ActuatorResponseParams{0.0, 0.0, 1.0}),
               std::invalid_argument);
  EXPECT_THROW(validateActuatorResponseParams(
                   ActuatorResponseParams{0.0, 0.1, 0.0}),
               std::invalid_argument);
  EXPECT_THROW(validateActuatorResponseParams(ActuatorResponseParams{
                   0.0, 0.1, std::numeric_limits<double>::quiet_NaN()}),
               std::invalid_argument);

  const ActuatorResponseParams valid{0.0, 0.1, 1.0};
  EXPECT_THROW(stepFopdt(std::numeric_limits<double>::quiet_NaN(), 1.0,
                         0.1, valid),
               std::invalid_argument);
  EXPECT_THROW(stepFopdt(0.0, 1.0, -0.1, valid),
               std::invalid_argument);
  EXPECT_THROW(stepFopdt(
                   0.0, std::numeric_limits<double>::max(), 0.1,
                   ActuatorResponseParams{0.0, 0.1, 2.0}),
               std::overflow_error);
  EXPECT_THROW(fopdtAcceleration(
                   0.0, std::numeric_limits<double>::infinity(), valid),
               std::invalid_argument);
}

TEST(MainlineFopdt, StatusKernelFailsClosedWithoutChangingOutput) {
  const FopdtChannelParams valid{0.1, 1.0};
  double next = 77.0;
  EXPECT_EQ(FopdtStepStatus::kInvalidParams,
            fopdtStep(0.0, 1.0, 0.1, FopdtChannelParams{0.0, 1.0},
                      next));
  EXPECT_DOUBLE_EQ(77.0, next);
  EXPECT_EQ(FopdtStepStatus::kInvalidState,
            fopdtStep(0.0, 1.0, -0.1, valid, next));
  EXPECT_DOUBLE_EQ(77.0, next);
  EXPECT_EQ(FopdtStepStatus::kNonFiniteOutput,
            fopdtStep(0.0, std::numeric_limits<double>::max(), 0.1,
                      FopdtChannelParams{0.1, 2.0}, next));
  EXPECT_DOUBLE_EQ(77.0, next);

  EXPECT_EQ(FopdtStepStatus::kOk,
            fopdtStep(0.0, 1.0, 0.1, valid, next));
  EXPECT_TRUE(std::isfinite(next));
  EXPECT_NE(77.0, next);
}

TEST(MainlineFopdt, StatusKernelDistinguishesInvalidStateAndOverflowPaths) {
  const FopdtChannelParams valid{0.1, 1.0};
  double next = 77.0;

  EXPECT_EQ(FopdtStepStatus::kInvalidState,
            fopdtStep(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.1,
                      valid, next));
  EXPECT_DOUBLE_EQ(77.0, next);
  EXPECT_EQ(FopdtStepStatus::kInvalidState,
            fopdtStep(0.0, 1.0, std::numeric_limits<double>::infinity(),
                      valid, next));
  EXPECT_DOUBLE_EQ(77.0, next);

  EXPECT_EQ(FopdtStepStatus::kNonFiniteOutput,
            fopdtStep(0.0, std::numeric_limits<double>::max(), 0.1,
                      FopdtChannelParams{0.1, 2.0}, next));
  EXPECT_DOUBLE_EQ(77.0, next);
  EXPECT_EQ(FopdtStepStatus::kNonFiniteOutput,
            fopdtStep(0.0, 1.0, std::numeric_limits<double>::max(),
                      FopdtChannelParams{std::numeric_limits<double>::min(),
                                         1.0},
                      next));
  EXPECT_DOUBLE_EQ(77.0, next);

  EXPECT_EQ(FopdtStepStatus::kOk,
            fopdtStep(3.0, -1.0, 0.0, valid, next));
  EXPECT_DOUBLE_EQ(3.0, next);
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
