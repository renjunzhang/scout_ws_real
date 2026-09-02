#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <type_traits>

#include "spmpc_local_planner/application/release_fallback_policy.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr double kDtSec = 1.0 / 30.0;

AuthoritativePublisherState publisherState(
    double linear_command, double angular_command,
    double linear_acceleration = 0.0,
    double angular_acceleration = 0.0) {
  AuthoritativePublisherState result;
  result.previous_linear_command = linear_command;
  result.previous_angular_command = angular_command;
  result.previous_linear_acceleration = linear_acceleration;
  result.previous_angular_acceleration = angular_acceleration;
  return result;
}

JerkLimitedFallbackParams fallbackParams(
    double maximum_linear_command, double maximum_angular_command,
    double maximum_linear_acceleration,
    double maximum_angular_acceleration, double maximum_linear_jerk,
    double maximum_angular_jerk) {
  JerkLimitedFallbackParams result;
  result.maximum_linear_command = maximum_linear_command;
  result.maximum_angular_command = maximum_angular_command;
  result.maximum_linear_acceleration = maximum_linear_acceleration;
  result.maximum_angular_acceleration = maximum_angular_acceleration;
  result.maximum_linear_jerk = maximum_linear_jerk;
  result.maximum_angular_jerk = maximum_angular_jerk;
  return result;
}

ReleaseFallbackOutput sentinelOutput() {
  ReleaseFallbackOutput result;
  result.command.linear = 11.0;
  result.command.angular = 12.0;
  result.authoritative_acceleration.linear = 13.0;
  result.authoritative_acceleration.angular = 14.0;
  result.stopped = true;
  return result;
}

void expectSentinelUnchanged(const ReleaseFallbackOutput& output) {
  EXPECT_EQ(11.0, output.command.linear);
  EXPECT_EQ(12.0, output.command.angular);
  EXPECT_EQ(13.0, output.authoritative_acceleration.linear);
  EXPECT_EQ(14.0, output.authoritative_acceleration.angular);
  EXPECT_TRUE(output.stopped);
}

void expectConstantJerkMap(double previous_command,
                           double previous_acceleration,
                           double next_command, double next_acceleration) {
  EXPECT_NEAR(previous_command +
                  0.5 * kDtSec *
                      (previous_acceleration + next_acceleration),
              next_command, 1e-14);
}

TEST(MainlineReleaseFallbackPolicy,
     UsesFrozenGridAndSlewsOutwardAccelerationAtJerkLimit) {
  static_assert(
      std::is_trivially_copyable<JerkLimitedFallbackPolicy>::value,
      "release fallback policy must contain no allocating ownership");
  EXPECT_DOUBLE_EQ(kDtSec, JerkLimitedFallbackPolicy::timeStepSeconds());

  const JerkLimitedFallbackPolicy policy(
      fallbackParams(1.0, 1.0, 0.6, 0.4, 3.0, 1.5));
  const AuthoritativePublisherState previous =
      publisherState(0.30, -0.20, 0.20, -0.10);
  ReleaseFallbackOutput output;

  ASSERT_EQ(ReleaseFallbackStatus::kOk,
            policy.computeNext(previous, output));
  EXPECT_NEAR(0.10, output.authoritative_acceleration.linear, 1e-14);
  EXPECT_NEAR(-0.05, output.authoritative_acceleration.angular, 1e-14);
  EXPECT_NEAR(-3.0,
              (output.authoritative_acceleration.linear -
               previous.previous_linear_acceleration) /
                  kDtSec,
              1e-13);
  EXPECT_NEAR(1.5,
              (output.authoritative_acceleration.angular -
               previous.previous_angular_acceleration) /
                  kDtSec,
              1e-13);
  EXPECT_GT(std::fabs(output.command.linear),
            std::fabs(previous.previous_linear_command));
  EXPECT_GT(std::fabs(output.command.angular),
            std::fabs(previous.previous_angular_command));
  expectConstantJerkMap(previous.previous_linear_command,
                        previous.previous_linear_acceleration,
                        output.command.linear,
                        output.authoritative_acceleration.linear);
  expectConstantJerkMap(previous.previous_angular_command,
                        previous.previous_angular_acceleration,
                        output.command.angular,
                        output.authoritative_acceleration.angular);
  EXPECT_FALSE(output.stopped);
}

TEST(MainlineReleaseFallbackPolicy,
     ChannelsBrakeIndependentlyFromRestWithoutAccelerationFlip) {
  const JerkLimitedFallbackPolicy policy(
      fallbackParams(1.0, 1.0, 0.6, 0.4, 3.0, 1.5));
  const AuthoritativePublisherState previous =
      publisherState(0.30, -0.20);
  ReleaseFallbackOutput output;

  ASSERT_EQ(ReleaseFallbackStatus::kOk,
            policy.computeNext(previous, output));
  EXPECT_NEAR(-0.10, output.authoritative_acceleration.linear, 1e-14);
  EXPECT_NEAR(0.05, output.authoritative_acceleration.angular, 1e-14);
  EXPECT_NEAR(0.30 - 0.05 * kDtSec, output.command.linear, 1e-14);
  EXPECT_NEAR(-0.20 + 0.025 * kDtSec, output.command.angular, 1e-14);
  EXPECT_LT(std::fabs(output.command.linear),
            std::fabs(previous.previous_linear_command));
  EXPECT_LT(std::fabs(output.command.angular),
            std::fabs(previous.previous_angular_command));
}

TEST(MainlineReleaseFallbackPolicy,
     RepeatedStepsRespectDynamicsAndReachExactZero) {
  constexpr double kMaximumLinearAcceleration = 0.6;
  constexpr double kMaximumAngularAcceleration = 0.4;
  constexpr double kMaximumLinearJerk = 3.0;
  constexpr double kMaximumAngularJerk = 2.0;
  const JerkLimitedFallbackPolicy policy(fallbackParams(
      1.0, 1.0, kMaximumLinearAcceleration,
      kMaximumAngularAcceleration, kMaximumLinearJerk,
      kMaximumAngularJerk));
  AuthoritativePublisherState state =
      publisherState(-0.21, 0.13, -0.30, 0.20);
  bool reached_zero = false;
  bool one_channel_stopped_first = false;

  for (std::size_t step = 0; step < 120; ++step) {
    const AuthoritativePublisherState previous = state;
    ReleaseFallbackOutput output;
    ASSERT_EQ(ReleaseFallbackStatus::kOk,
              policy.computeNext(previous, output));

    EXPECT_GE(output.command.linear * previous.previous_linear_command,
              -1e-15);
    EXPECT_GE(output.command.angular * previous.previous_angular_command,
              -1e-15);
    EXPECT_LE(std::fabs(output.command.linear), 1.0);
    EXPECT_LE(std::fabs(output.command.angular), 1.0);
    EXPECT_LE(std::fabs(output.authoritative_acceleration.linear),
              kMaximumLinearAcceleration + 1e-14);
    EXPECT_LE(std::fabs(output.authoritative_acceleration.angular),
              kMaximumAngularAcceleration + 1e-14);
    EXPECT_LE(std::fabs(output.authoritative_acceleration.linear -
                        previous.previous_linear_acceleration),
              kMaximumLinearJerk * kDtSec + 1e-14);
    EXPECT_LE(std::fabs(output.authoritative_acceleration.angular -
                        previous.previous_angular_acceleration),
              kMaximumAngularJerk * kDtSec + 1e-14);
    expectConstantJerkMap(previous.previous_linear_command,
                          previous.previous_linear_acceleration,
                          output.command.linear,
                          output.authoritative_acceleration.linear);
    expectConstantJerkMap(previous.previous_angular_command,
                          previous.previous_angular_acceleration,
                          output.command.angular,
                          output.authoritative_acceleration.angular);

    const double linear_direction =
        previous.previous_linear_command >= 0.0 ? 1.0 : -1.0;
    const double angular_direction =
        previous.previous_angular_command >= 0.0 ? 1.0 : -1.0;
    if (linear_direction * previous.previous_linear_acceleration <= 0.0) {
      EXPECT_LE(std::fabs(output.command.linear),
                std::fabs(previous.previous_linear_command) + 1e-14);
    }
    if (angular_direction * previous.previous_angular_acceleration <= 0.0) {
      EXPECT_LE(std::fabs(output.command.angular),
                std::fabs(previous.previous_angular_command) + 1e-14);
    }

    if ((output.command.linear == 0.0 &&
         output.authoritative_acceleration.linear == 0.0) !=
        (output.command.angular == 0.0 &&
         output.authoritative_acceleration.angular == 0.0)) {
      one_channel_stopped_first = true;
      EXPECT_FALSE(output.stopped);
    }

    state.previous_linear_command = output.command.linear;
    state.previous_angular_command = output.command.angular;
    state.previous_linear_acceleration =
        output.authoritative_acceleration.linear;
    state.previous_angular_acceleration =
        output.authoritative_acceleration.angular;
    if (output.stopped) {
      reached_zero = true;
      break;
    }
  }

  EXPECT_TRUE(reached_zero);
  EXPECT_TRUE(one_channel_stopped_first);
  EXPECT_EQ(0.0, state.previous_linear_command);
  EXPECT_EQ(0.0, state.previous_angular_command);
  EXPECT_EQ(0.0, state.previous_linear_acceleration);
  EXPECT_EQ(0.0, state.previous_angular_acceleration);
}

TEST(MainlineReleaseFallbackPolicy,
     ExactTerminalStepStopsWithZeroCommandAndAcceleration) {
  const JerkLimitedFallbackPolicy policy(
      fallbackParams(1.0, 1.0, 0.6, 0.4, 3.0, 2.0));
  const double previous_acceleration = -3.0 * kDtSec;
  const double exact_distance =
      -0.5 * kDtSec * previous_acceleration;
  ReleaseFallbackOutput output;

  ASSERT_EQ(ReleaseFallbackStatus::kOk,
            policy.computeNext(
                publisherState(exact_distance, 0.0,
                               previous_acceleration, 0.0),
                output));
  EXPECT_EQ(0.0, output.command.linear);
  EXPECT_EQ(0.0, output.command.angular);
  EXPECT_EQ(0.0, output.authoritative_acceleration.linear);
  EXPECT_EQ(0.0, output.authoritative_acceleration.angular);
  EXPECT_TRUE(output.stopped);
}

TEST(MainlineReleaseFallbackPolicy,
     RecoverableStateGridRemainsAdmissibleUntilStopped) {
  constexpr double kMaximumAcceleration = 0.6;
  constexpr double kMaximumJerk = 3.0;
  const JerkLimitedFallbackPolicy policy(fallbackParams(
      1.0, 1.0, kMaximumAcceleration, kMaximumAcceleration,
      kMaximumJerk, kMaximumJerk));
  const std::array<double, 8> commands =
      {{-0.9, -0.3, -0.1, -0.02, 0.02, 0.1, 0.3, 0.9}};
  const std::array<double, 7> accelerations =
      {{-0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6}};
  std::size_t recovered_count = 0;
  std::size_t rejected_count = 0;

  for (const double initial_command : commands) {
    for (const double initial_acceleration : accelerations) {
      SCOPED_TRACE(::testing::Message()
                   << "q0=" << initial_command
                   << ", a0=" << initial_acceleration);
      AuthoritativePublisherState state =
          publisherState(initial_command, 0.0, initial_acceleration, 0.0);
      ReleaseFallbackOutput first = sentinelOutput();
      const ReleaseFallbackStatus first_status =
          policy.computeNext(state, first);
      if (first_status ==
          ReleaseFallbackStatus::kUnrecoverablePublisherState) {
        ++rejected_count;
        expectSentinelUnchanged(first);
        continue;
      }
      ASSERT_EQ(ReleaseFallbackStatus::kOk, first_status);

      ReleaseFallbackOutput output = first;
      for (std::size_t step = 0; step < 160; ++step) {
        const double previous_command = state.previous_linear_command;
        const double previous_acceleration =
            state.previous_linear_acceleration;
        if (step != 0) {
          ASSERT_EQ(ReleaseFallbackStatus::kOk,
                    policy.computeNext(state, output));
        }
        EXPECT_GE(output.command.linear * previous_command, -1e-15);
        EXPECT_LE(std::fabs(output.command.linear), 1.0 + 1e-14);
        EXPECT_LE(std::fabs(output.authoritative_acceleration.linear),
                  kMaximumAcceleration + 1e-14);
        EXPECT_LE(std::fabs(output.authoritative_acceleration.linear -
                            previous_acceleration),
                  kMaximumJerk * kDtSec + 1e-14);
        expectConstantJerkMap(
            previous_command, previous_acceleration,
            output.command.linear,
            output.authoritative_acceleration.linear);

        state.previous_linear_command = output.command.linear;
        state.previous_linear_acceleration =
            output.authoritative_acceleration.linear;
        if (output.stopped) {
          ++recovered_count;
          break;
        }
        ASSERT_LT(step + 1, 160u);
      }
    }
  }

  EXPECT_GT(recovered_count, 0u);
  EXPECT_GT(rejected_count, 0u);
}

TEST(MainlineReleaseFallbackPolicy,
     CommandLimitWithOutwardAccelerationIsUnrecoverable) {
  const JerkLimitedFallbackPolicy policy(
      fallbackParams(1.0, 1.0, 0.6, 0.4, 3.0, 2.0));
  ReleaseFallbackOutput output = sentinelOutput();

  EXPECT_EQ(ReleaseFallbackStatus::kUnrecoverablePublisherState,
            policy.computeNext(
                publisherState(1.0, 0.2, 0.1, 0.0), output));
  expectSentinelUnchanged(output);
}

TEST(MainlineReleaseFallbackPolicy, ZeroStateRemainsStopped) {
  const JerkLimitedFallbackPolicy policy(
      fallbackParams(1.0, 1.0, 0.6, 0.4, 3.0, 2.0));
  ReleaseFallbackOutput output;

  ASSERT_EQ(ReleaseFallbackStatus::kOk,
            policy.computeNext(publisherState(0.0, -0.0), output));
  EXPECT_EQ(0.0, output.command.linear);
  EXPECT_EQ(0.0, output.command.angular);
  EXPECT_EQ(0.0, output.authoritative_acceleration.linear);
  EXPECT_EQ(0.0, output.authoritative_acceleration.angular);
  EXPECT_TRUE(output.stopped);
}

TEST(MainlineReleaseFallbackPolicy,
     UnrecoverableNearZeroStateFailsAtomically) {
  const JerkLimitedFallbackPolicy policy(
      fallbackParams(1.0, 1.0, 0.6, 0.4, 3.0, 2.0));
  ReleaseFallbackOutput output = sentinelOutput();

  EXPECT_EQ(ReleaseFallbackStatus::kUnrecoverablePublisherState,
            policy.computeNext(
                publisherState(0.01, 0.20, -0.60, 0.0), output));
  expectSentinelUnchanged(output);
}

TEST(MainlineReleaseFallbackPolicy,
     NegativeSubUlpStoppingMarginIsNeverAccepted) {
  const JerkLimitedFallbackPolicy policy(
      fallbackParams(1.0, 1.0, 0.6, 0.4, 3.0, 2.0));
  ReleaseFallbackOutput output = sentinelOutput();

  EXPECT_EQ(ReleaseFallbackStatus::kUnrecoverablePublisherState,
            policy.computeNext(
                publisherState(0.059999999999985995, 0.20,
                               -0.60, 0.0),
                output));
  expectSentinelUnchanged(output);
}

TEST(MainlineReleaseFallbackPolicy,
     ZeroCommandWithResidualAccelerationIsUnrecoverable) {
  const JerkLimitedFallbackPolicy policy(
      fallbackParams(1.0, 1.0, 0.6, 0.4, 3.0, 2.0));
  ReleaseFallbackOutput output = sentinelOutput();

  EXPECT_EQ(ReleaseFallbackStatus::kUnrecoverablePublisherState,
            policy.computeNext(
                publisherState(0.0, 0.20, 0.10, 0.0), output));
  expectSentinelUnchanged(output);
}

TEST(MainlineReleaseFallbackPolicy,
     InvalidParametersReturnStatusWithoutChangingOutput) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<JerkLimitedFallbackParams, 8> invalid = {{
      {0.0, 1.0, 1.0, 1.0, 1.0, 1.0},
      {1.0, -1.0, 1.0, 1.0, 1.0, 1.0},
      {1.0, 1.0, 0.0, 1.0, 1.0, 1.0},
      {1.0, 1.0, 1.0, -1.0, 1.0, 1.0},
      {1.0, 1.0, 1.0, 1.0, 0.0, 1.0},
      {1.0, 1.0, 1.0, 1.0, 1.0, -1.0},
      {nan, 1.0, 1.0, 1.0, 1.0, 1.0},
      {1.0, 1.0, 1.0, 1.0, infinity, 1.0},
  }};

  for (const JerkLimitedFallbackParams& parameters : invalid) {
    const JerkLimitedFallbackPolicy policy(parameters);
    ReleaseFallbackOutput output = sentinelOutput();
    EXPECT_EQ(ReleaseFallbackStatus::kInvalidParameters,
              policy.computeNext(publisherState(0.2, -0.1), output));
    expectSentinelUnchanged(output);
  }
}

TEST(MainlineReleaseFallbackPolicy,
     InvalidPublisherStateReturnsStatusWithoutChangingOutput) {
  const JerkLimitedFallbackPolicy policy(
      fallbackParams(1.0, 1.0, 0.6, 0.4, 3.0, 2.0));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<AuthoritativePublisherState, 12> invalid = {{
      publisherState(nan, 0.0, 0.0, 0.0),
      publisherState(0.0, nan, 0.0, 0.0),
      publisherState(0.0, 0.0, nan, 0.0),
      publisherState(0.0, 0.0, 0.0, nan),
      publisherState(infinity, 0.0, 0.0, 0.0),
      publisherState(0.0, -infinity, 0.0, 0.0),
      publisherState(0.0, 0.0, infinity, 0.0),
      publisherState(0.0, 0.0, 0.0, -infinity),
      publisherState(1.01, 0.0, 0.0, 0.0),
      publisherState(0.0, -1.01, 0.0, 0.0),
      publisherState(0.2, 0.0, 0.61, 0.0),
      publisherState(0.0, 0.2, 0.0, -0.41),
  }};

  for (const AuthoritativePublisherState& state : invalid) {
    ReleaseFallbackOutput output = sentinelOutput();
    EXPECT_EQ(ReleaseFallbackStatus::kInvalidPublisherState,
              policy.computeNext(state, output));
    expectSentinelUnchanged(output);
  }
}

TEST(MainlineReleaseFallbackPolicy,
     NumericalRangeFailureReturnsStatusWithoutChangingOutput) {
  const double denormal = std::numeric_limits<double>::denorm_min();
  const JerkLimitedFallbackPolicy underflow_policy(
      fallbackParams(1.0, 1.0, 1.0, 1.0, denormal, denormal));
  ReleaseFallbackOutput output = sentinelOutput();

  EXPECT_EQ(ReleaseFallbackStatus::kNumericalRangeError,
            underflow_policy.computeNext(
                publisherState(0.2, -0.1), output));
  expectSentinelUnchanged(output);
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
