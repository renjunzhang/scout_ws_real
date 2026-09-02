#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/execution/actuator_discrete_model.h"
#include "stage2_execution_golden_generated.hpp"

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr double kTolerance = 1e-12;

// Focused edge/failure tests retain local synthetic inputs.  The complete-map
// golden scenario below exclusively consumes the generated fixture types.
ActuatorDiscreteConfig makeSyntheticConfig() {
  ActuatorDiscreteConfig config;
  config.dt_sec = 1.0;
  config.maximum_linear_delay_sec = 3.0;
  config.maximum_angular_delay_sec = 3.0;
  config.linear_delay_sec = 0.5;
  config.angular_delay_sec = 0.25;
  config.integer_snap_tolerance_ratio = kTolerance;
  config.duration_tolerance_sec = kTolerance;
  return config;
}

ZohPlantParams makeSyntheticPlantParams() {
  ZohPlantParams params;
  params.linear_actuator = FopdtChannelParams{2.0, 1.0};
  params.angular_actuator = FopdtChannelParams{4.0, 1.0};
  params.liquid = LiquidModalParams{1.3, 0.2, 1.1, 0.9};
  return params;
}

using SyntheticModel = ActuatorDiscreteModel<4, 4>;
using SyntheticState = SyntheticModel::State;
using SyntheticResult = SyntheticModel::Result;

using GoldenModel = ActuatorDiscreteModel<
    stage2_execution_golden::kLinearSelectorWidth,
    stage2_execution_golden::kAngularSelectorWidth>;
using GoldenState = GoldenModel::State;
using GoldenResult = GoldenModel::Result;

ActuatorDiscreteConfig makeGoldenConfig() {
  const auto& source = stage2_execution_golden::kConfig;
  ActuatorDiscreteConfig config;
  config.dt_sec = source.dt_sec;
  config.maximum_linear_delay_sec = source.maximum_linear_delay_sec;
  config.maximum_angular_delay_sec = source.maximum_angular_delay_sec;
  config.linear_delay_sec = source.linear_delay_sec;
  config.angular_delay_sec = source.angular_delay_sec;
  config.integer_snap_tolerance_ratio = source.integer_snap_tolerance_ratio;
  config.duration_tolerance_sec = source.duration_tolerance_sec;
  return config;
}

ZohPlantParams makeGoldenPlantParams() {
  const auto& linear = stage2_execution_golden::kLinearActuator;
  const auto& angular = stage2_execution_golden::kAngularActuator;
  const auto& liquid = stage2_execution_golden::kLiquid;
  ZohPlantParams params;
  params.linear_actuator = FopdtChannelParams{linear.tau_sec, linear.gain};
  params.angular_actuator = FopdtChannelParams{angular.tau_sec, angular.gain};
  params.liquid = LiquidModalParams{
      liquid.natural_frequency_rad_per_sec,
      liquid.damping_ratio,
      liquid.longitudinal_coupling,
      liquid.lateral_coupling};
  return params;
}

GoldenState makeGoldenState(
    const stage2_execution_golden::GoldenState& source) {
  GoldenState state;
  state.physical.pose = PlanarPoseState{
      source.physical.pose.x, source.physical.pose.y,
      source.physical.pose.heading};
  state.physical.actual = ActualMotionState{
      source.physical.actual.linear_velocity,
      source.physical.actual.angular_velocity};
  state.physical.liquid = LiquidModalState{
      source.physical.liquid.eta_x, source.physical.liquid.eta_x_dot,
      source.physical.liquid.eta_y, source.physical.liquid.eta_y_dot};
  state.progress = source.progress;
  state.publisher = AuthoritativePublisherState{
      source.publisher.previous_linear_command,
      source.publisher.previous_angular_command,
      source.publisher.previous_linear_acceleration,
      source.publisher.previous_angular_acceleration};
  state.linear_older = source.linear_older;
  state.angular_older = source.angular_older;
  return state;
}

SyntheticState makeSyntheticState() {
  SyntheticState state;
  state.progress = 10.0;
  state.publisher.previous_linear_command = 1.0;
  state.publisher.previous_angular_command = -1.0;
  state.publisher.previous_linear_acceleration = 2.0;
  state.publisher.previous_angular_acceleration = 0.5;
  state.linear_older = {{3.0, -4.0}};
  state.angular_older = {{2.0, -3.0}};
  return state;
}

SyntheticResult makeSentinel() {
  SyntheticResult result;
  result.issued = IssuedCommand{81.0, 82.0, 83.0, 84.0};
  for (std::size_t slot = 0; slot < result.segments.size(); ++slot) {
    result.segments[slot] =
        ZohTargetSegment{90.0 + static_cast<double>(slot),
                         100.0 + static_cast<double>(slot),
                         110.0 + static_cast<double>(slot)};
  }
  result.next_state.physical.pose = PlanarPoseState{121.0, 122.0, 123.0};
  result.next_state.physical.actual = ActualMotionState{124.0, 125.0};
  result.next_state.physical.liquid =
      LiquidModalState{126.0, 127.0, 128.0, 129.0};
  result.next_state.progress = 130.0;
  result.next_state.publisher =
      AuthoritativePublisherState{131.0, 132.0, 133.0, 134.0};
  result.next_state.linear_older = {{135.0, 136.0}};
  result.next_state.angular_older = {{137.0, 138.0}};
  return result;
}

template <std::size_t LinearWidth, std::size_t AngularWidth>
void expectStateDoubleEq(
    const PreIssueActuatorState<LinearWidth, AngularWidth>& expected,
    const PreIssueActuatorState<LinearWidth, AngularWidth>& actual) {
  EXPECT_DOUBLE_EQ(expected.physical.pose.x, actual.physical.pose.x);
  EXPECT_DOUBLE_EQ(expected.physical.pose.y, actual.physical.pose.y);
  EXPECT_DOUBLE_EQ(expected.physical.pose.heading,
                   actual.physical.pose.heading);
  EXPECT_DOUBLE_EQ(expected.physical.actual.linear_velocity,
                   actual.physical.actual.linear_velocity);
  EXPECT_DOUBLE_EQ(expected.physical.actual.angular_velocity,
                   actual.physical.actual.angular_velocity);
  EXPECT_DOUBLE_EQ(expected.physical.liquid.eta_x,
                   actual.physical.liquid.eta_x);
  EXPECT_DOUBLE_EQ(expected.physical.liquid.eta_x_dot,
                   actual.physical.liquid.eta_x_dot);
  EXPECT_DOUBLE_EQ(expected.physical.liquid.eta_y,
                   actual.physical.liquid.eta_y);
  EXPECT_DOUBLE_EQ(expected.physical.liquid.eta_y_dot,
                   actual.physical.liquid.eta_y_dot);
  EXPECT_DOUBLE_EQ(expected.progress, actual.progress);
  EXPECT_DOUBLE_EQ(expected.publisher.previous_linear_command,
                   actual.publisher.previous_linear_command);
  EXPECT_DOUBLE_EQ(expected.publisher.previous_angular_command,
                   actual.publisher.previous_angular_command);
  EXPECT_DOUBLE_EQ(expected.publisher.previous_linear_acceleration,
                   actual.publisher.previous_linear_acceleration);
  EXPECT_DOUBLE_EQ(expected.publisher.previous_angular_acceleration,
                   actual.publisher.previous_angular_acceleration);
  EXPECT_EQ(expected.linear_older, actual.linear_older);
  EXPECT_EQ(expected.angular_older, actual.angular_older);
}

template <std::size_t LinearWidth, std::size_t AngularWidth>
void expectResultDoubleEq(
    const ActuatorDiscreteStepResult<LinearWidth, AngularWidth>& expected,
    const ActuatorDiscreteStepResult<LinearWidth, AngularWidth>& actual) {
  EXPECT_DOUBLE_EQ(expected.issued.linear_command,
                   actual.issued.linear_command);
  EXPECT_DOUBLE_EQ(expected.issued.angular_command,
                   actual.issued.angular_command);
  EXPECT_DOUBLE_EQ(expected.issued.linear_acceleration,
                   actual.issued.linear_acceleration);
  EXPECT_DOUBLE_EQ(expected.issued.angular_acceleration,
                   actual.issued.angular_acceleration);
  for (std::size_t slot = 0; slot < expected.segments.size(); ++slot) {
    EXPECT_DOUBLE_EQ(expected.segments[slot].duration_sec,
                     actual.segments[slot].duration_sec);
    EXPECT_DOUBLE_EQ(expected.segments[slot].linear_target,
                     actual.segments[slot].linear_target);
    EXPECT_DOUBLE_EQ(expected.segments[slot].angular_target,
                     actual.segments[slot].angular_target);
  }
  expectStateDoubleEq(expected.next_state, actual.next_state);
}

template <std::size_t LinearWidth, std::size_t AngularWidth>
void expectStateNear(
    const PreIssueActuatorState<LinearWidth, AngularWidth>& expected,
    const PreIssueActuatorState<LinearWidth, AngularWidth>& actual,
    double tolerance) {
  EXPECT_NEAR(expected.physical.pose.x, actual.physical.pose.x, tolerance);
  EXPECT_NEAR(expected.physical.pose.y, actual.physical.pose.y, tolerance);
  EXPECT_NEAR(expected.physical.pose.heading, actual.physical.pose.heading,
              tolerance);
  EXPECT_NEAR(expected.physical.actual.linear_velocity,
              actual.physical.actual.linear_velocity, tolerance);
  EXPECT_NEAR(expected.physical.actual.angular_velocity,
              actual.physical.actual.angular_velocity, tolerance);
  EXPECT_NEAR(expected.physical.liquid.eta_x,
              actual.physical.liquid.eta_x, tolerance);
  EXPECT_NEAR(expected.physical.liquid.eta_x_dot,
              actual.physical.liquid.eta_x_dot, tolerance);
  EXPECT_NEAR(expected.physical.liquid.eta_y,
              actual.physical.liquid.eta_y, tolerance);
  EXPECT_NEAR(expected.physical.liquid.eta_y_dot,
              actual.physical.liquid.eta_y_dot, tolerance);
  EXPECT_NEAR(expected.progress, actual.progress, tolerance);
  EXPECT_NEAR(expected.publisher.previous_linear_command,
              actual.publisher.previous_linear_command, tolerance);
  EXPECT_NEAR(expected.publisher.previous_angular_command,
              actual.publisher.previous_angular_command, tolerance);
  EXPECT_NEAR(expected.publisher.previous_linear_acceleration,
              actual.publisher.previous_linear_acceleration, tolerance);
  EXPECT_NEAR(expected.publisher.previous_angular_acceleration,
              actual.publisher.previous_angular_acceleration, tolerance);
  EXPECT_EQ(expected.linear_older, actual.linear_older);
  EXPECT_EQ(expected.angular_older, actual.angular_older);
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

TEST(MainlineActuatorDiscreteModel, FreezesValidIndependentSchedules) {
  const SyntheticModel model(makeSyntheticConfig(),
                             makeSyntheticPlantParams());
  EXPECT_EQ(0u, model.linearSchedule().integer_delay_steps);
  EXPECT_DOUBLE_EQ(0.5, model.linearSchedule().fractional_beta);
  EXPECT_EQ(0u, model.angularSchedule().integer_delay_steps);
  EXPECT_DOUBLE_EQ(0.25, model.angularSchedule().fractional_beta);
  ASSERT_TRUE(model.combinedSchedule().valid(1.0, kTolerance));
  EXPECT_DOUBLE_EQ(0.25, model.combinedSchedule().duration[0]);
  EXPECT_DOUBLE_EQ(0.25, model.combinedSchedule().duration[1]);
  EXPECT_DOUBLE_EQ(0.5, model.combinedSchedule().duration[2]);

  EXPECT_THROW((ActuatorDiscreteModel<3, 4>(makeSyntheticConfig(),
                                            makeSyntheticPlantParams())),
               std::invalid_argument);
  ActuatorDiscreteConfig invalid = makeSyntheticConfig();
  invalid.linear_delay_sec = 4.0;
  EXPECT_THROW((SyntheticModel(invalid, makeSyntheticPlantParams())),
               std::invalid_argument);
  ZohPlantParams invalid_plant = makeSyntheticPlantParams();
  invalid_plant.liquid.damping_ratio = -1.0;
  EXPECT_THROW((SyntheticModel(makeSyntheticConfig(), invalid_plant)),
               std::invalid_argument);
}

TEST(MainlineActuatorDiscreteModel, MatchesIndependentCompleteMapGolden) {
  const GoldenModel model(makeGoldenConfig(), makeGoldenPlantParams());
  const GoldenState state =
      makeGoldenState(stage2_execution_golden::kInitialState);
  const IssueControl control{
      stage2_execution_golden::kControl.linear_jerk,
      stage2_execution_golden::kControl.angular_jerk,
      stage2_execution_golden::kControl.progress_velocity};
  GoldenResult output;
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(state, control, output));

  const auto& expected_issued = stage2_execution_golden::kExpectedIssued;
  const double tolerance = stage2_execution_golden::kAbsoluteTolerance;
  EXPECT_NEAR(expected_issued.linear_command, output.issued.linear_command,
              tolerance);
  EXPECT_NEAR(expected_issued.angular_command, output.issued.angular_command,
              tolerance);
  EXPECT_NEAR(expected_issued.linear_acceleration,
              output.issued.linear_acceleration, tolerance);
  EXPECT_NEAR(expected_issued.angular_acceleration,
              output.issued.angular_acceleration, tolerance);
  for (std::size_t slot = 0; slot < output.segments.size(); ++slot) {
    const auto& expected = stage2_execution_golden::kExpectedSegments[slot];
    EXPECT_NEAR(expected.duration_sec, output.segments[slot].duration_sec,
                tolerance);
    EXPECT_NEAR(expected.linear_target, output.segments[slot].linear_target,
                tolerance);
    EXPECT_NEAR(expected.angular_target, output.segments[slot].angular_target,
                tolerance);
  }

  expectStateNear(
      makeGoldenState(stage2_execution_golden::kExpectedNextState),
      output.next_state, tolerance);
  expectStateDoubleEq(
      makeGoldenState(stage2_execution_golden::kInitialState), state);
}

TEST(MainlineActuatorDiscreteModel, CurrentIssueCannotLeakBeforeOneStepDelay) {
  ActuatorDiscreteConfig config = makeSyntheticConfig();
  config.maximum_linear_delay_sec = 1.0;
  config.maximum_angular_delay_sec = 1.0;
  config.linear_delay_sec = 1.0;
  config.angular_delay_sec = 1.0;
  const ActuatorDiscreteModel<2, 2> model(config,
                                         makeSyntheticPlantParams());
  ActuatorDiscreteModel<2, 2>::State state;
  state.publisher.previous_linear_command = 0.5;
  state.publisher.previous_angular_command = -0.4;

  ActuatorDiscreteModel<2, 2>::Result first;
  ActuatorDiscreteModel<2, 2>::Result second;
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(state, IssueControl{0.0, 0.0, 0.2}, first));
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(state, IssueControl{10.0, -8.0, 0.2}, second));
  EXPECT_NE(first.issued.linear_command, second.issued.linear_command);
  EXPECT_NE(first.issued.angular_command, second.issued.angular_command);
  EXPECT_DOUBLE_EQ(0.5, first.segments[0].linear_target);
  EXPECT_DOUBLE_EQ(0.5, second.segments[0].linear_target);
  EXPECT_DOUBLE_EQ(-0.4, first.segments[0].angular_target);
  EXPECT_DOUBLE_EQ(-0.4, second.segments[0].angular_target);
  expectPhysicalDoubleEq(first.next_state.physical,
                         second.next_state.physical);
  EXPECT_DOUBLE_EQ(first.next_state.progress, second.next_state.progress);
  EXPECT_EQ(first.next_state.linear_older,
            second.next_state.linear_older);
  EXPECT_EQ(first.next_state.angular_older,
            second.next_state.angular_older);
  EXPECT_NE(first.next_state.publisher.previous_linear_command,
            second.next_state.publisher.previous_linear_command);
}

TEST(MainlineActuatorDiscreteModel, MaximumIntegerDelayUsesOldestPreShiftTap) {
  ActuatorDiscreteConfig config = makeSyntheticConfig();
  config.linear_delay_sec = 3.0;
  config.angular_delay_sec = 3.0;
  const SyntheticModel model(config, makeSyntheticPlantParams());
  const SyntheticState state = makeSyntheticState();

  SyntheticResult first;
  SyntheticResult second;
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(state, IssueControl{0.0, 0.0, 0.2}, first));
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(state, IssueControl{10.0, -8.0, 0.2}, second));
  EXPECT_DOUBLE_EQ(-4.0, first.segments[0].linear_target);
  EXPECT_DOUBLE_EQ(-4.0, second.segments[0].linear_target);
  EXPECT_DOUBLE_EQ(-3.0, first.segments[0].angular_target);
  EXPECT_DOUBLE_EQ(-3.0, second.segments[0].angular_target);
  EXPECT_DOUBLE_EQ(1.0, first.segments[0].duration_sec);
  expectPhysicalDoubleEq(first.next_state.physical,
                         second.next_state.physical);
  EXPECT_EQ((std::array<double, 2>{{1.0, 3.0}}),
            first.next_state.linear_older);
  EXPECT_EQ((std::array<double, 2>{{-1.0, 2.0}}),
            first.next_state.angular_older);
}

TEST(MainlineActuatorDiscreteModel, ZeroDelayUsesCurrentIssuedCommand) {
  ActuatorDiscreteConfig config = makeSyntheticConfig();
  config.maximum_linear_delay_sec = 0.0;
  config.maximum_angular_delay_sec = 0.0;
  config.linear_delay_sec = 0.0;
  config.angular_delay_sec = 0.0;
  const ActuatorDiscreteModel<1, 1> model(config,
                                         makeSyntheticPlantParams());
  ActuatorDiscreteModel<1, 1>::State state;
  state.publisher.previous_linear_command = 1.0;
  state.publisher.previous_angular_command = -1.0;
  state.publisher.previous_linear_acceleration = 2.0;
  state.publisher.previous_angular_acceleration = 0.5;
  ActuatorDiscreteModel<1, 1>::Result output;
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(state, IssueControl{4.0, 0.0, 0.0}, output));
  EXPECT_DOUBLE_EQ(5.0, output.segments[0].linear_target);
  EXPECT_DOUBLE_EQ(-0.5, output.segments[0].angular_target);
  EXPECT_DOUBLE_EQ(1.0, output.segments[0].duration_sec);
  EXPECT_DOUBLE_EQ(0.0, output.segments[1].duration_sec);
  EXPECT_DOUBLE_EQ(0.0, output.segments[2].duration_sec);
}

TEST(MainlineActuatorDiscreteModel, SupportsIndependentSelectorWidths) {
  ActuatorDiscreteConfig config = makeSyntheticConfig();
  config.maximum_linear_delay_sec = 0.0;
  config.linear_delay_sec = 0.0;
  config.angular_delay_sec = 2.5;
  const ActuatorDiscreteModel<1, 4> model(config,
                                         makeSyntheticPlantParams());
  ActuatorDiscreteModel<1, 4>::State state;
  state.publisher.previous_linear_command = 1.0;
  state.publisher.previous_angular_command = -1.0;
  state.publisher.previous_linear_acceleration = 2.0;
  state.publisher.previous_angular_acceleration = 0.5;
  state.angular_older = {{2.0, -3.0}};

  ActuatorDiscreteModel<1, 4>::Result output;
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(state, IssueControl{4.0, 0.0, 0.0}, output));
  EXPECT_DOUBLE_EQ(0.5, output.segments[0].duration_sec);
  EXPECT_DOUBLE_EQ(0.5, output.segments[1].duration_sec);
  EXPECT_DOUBLE_EQ(0.0, output.segments[2].duration_sec);
  EXPECT_DOUBLE_EQ(5.0, output.segments[0].linear_target);
  EXPECT_DOUBLE_EQ(5.0, output.segments[1].linear_target);
  EXPECT_DOUBLE_EQ(-3.0, output.segments[0].angular_target);
  EXPECT_DOUBLE_EQ(2.0, output.segments[1].angular_target);
  EXPECT_EQ((std::array<double, 2>{{-1.0, 2.0}}),
            output.next_state.angular_older);
}

TEST(MainlineActuatorDiscreteModel, ConsecutiveStepsShiftOnlyAfterPropagation) {
  const SyntheticModel model(makeSyntheticConfig(),
                             makeSyntheticPlantParams());
  SyntheticResult first;
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(makeSyntheticState(), IssueControl{4.0, 0.0, 0.7},
                       first));
  SyntheticResult second;
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(first.next_state, IssueControl{0.0, 0.0, 0.7},
                       second));
  EXPECT_DOUBLE_EQ(11.0, second.issued.linear_command);
  EXPECT_DOUBLE_EQ(0.0, second.issued.angular_command);
  EXPECT_EQ((std::array<double, 2>{{5.0, 1.0}}),
            second.next_state.linear_older);
  EXPECT_EQ((std::array<double, 2>{{-0.5, -1.0}}),
            second.next_state.angular_older);
}

TEST(MainlineActuatorDiscreteModel, RejectsFailuresWithoutChangingOutput) {
  const SyntheticModel model(makeSyntheticConfig(),
                             makeSyntheticPlantParams());
  const SyntheticResult sentinel = makeSentinel();
  SyntheticResult output = sentinel;
  SyntheticState state = makeSyntheticState();

  state.linear_older[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ActuatorDiscreteStepStatus::kInvalidState,
            model.step(state, IssueControl{}, output));
  expectResultDoubleEq(sentinel, output);

  state = makeSyntheticState();
  EXPECT_EQ(ActuatorDiscreteStepStatus::kInvalidControl,
            model.step(state, IssueControl{0.0, 0.0, -0.1}, output));
  expectResultDoubleEq(sentinel, output);
  EXPECT_EQ(ActuatorDiscreteStepStatus::kInvalidControl,
            model.step(
                state,
                IssueControl{std::numeric_limits<double>::infinity(), 0.0,
                             0.0},
                output));
  expectResultDoubleEq(sentinel, output);

  state.publisher.previous_linear_acceleration =
      std::numeric_limits<double>::max();
  EXPECT_EQ(ActuatorDiscreteStepStatus::kIssueMapFailure,
            model.step(
                state,
                IssueControl{std::numeric_limits<double>::max(), 0.0, 0.0},
                output));
  expectResultDoubleEq(sentinel, output);

  state = makeSyntheticState();
  state.physical.actual = ActualMotionState{
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max()};
  EXPECT_EQ(ActuatorDiscreteStepStatus::kPlantPropagationFailure,
            model.step(state, IssueControl{}, output));
  expectResultDoubleEq(sentinel, output);

  state = makeSyntheticState();
  state.progress = std::numeric_limits<double>::max();
  EXPECT_EQ(ActuatorDiscreteStepStatus::kNonFiniteOutput,
            model.step(
                state,
                IssueControl{0.0, 0.0,
                             std::numeric_limits<double>::max()},
                output));
  expectResultDoubleEq(sentinel, output);
}

}  // namespace
}  // namespace mainline
}  // namespace spmpc_local_planner
