#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "spmpc_local_planner/execution/actuator_discrete_model.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr double kTolerance = 1e-12;

ActuatorDiscreteConfig makeConfig() {
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

ZohPlantParams makePlantParams() {
  ZohPlantParams params;
  params.linear_actuator = FopdtChannelParams{2.0, 1.0};
  params.angular_actuator = FopdtChannelParams{4.0, 1.0};
  params.liquid = LiquidModalParams{1.3, 0.2, 1.1, 0.9};
  return params;
}

using SyntheticModel = ActuatorDiscreteModel<4, 4>;
using SyntheticState = SyntheticModel::State;
using SyntheticResult = SyntheticModel::Result;

SyntheticState makeState() {
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
  const SyntheticModel model(makeConfig(), makePlantParams());
  EXPECT_EQ(0u, model.linearSchedule().integer_delay_steps);
  EXPECT_DOUBLE_EQ(0.5, model.linearSchedule().fractional_beta);
  EXPECT_EQ(0u, model.angularSchedule().integer_delay_steps);
  EXPECT_DOUBLE_EQ(0.25, model.angularSchedule().fractional_beta);
  ASSERT_TRUE(model.combinedSchedule().valid(1.0, kTolerance));
  EXPECT_DOUBLE_EQ(0.25, model.combinedSchedule().duration[0]);
  EXPECT_DOUBLE_EQ(0.25, model.combinedSchedule().duration[1]);
  EXPECT_DOUBLE_EQ(0.5, model.combinedSchedule().duration[2]);

  EXPECT_THROW((ActuatorDiscreteModel<3, 4>(makeConfig(),
                                            makePlantParams())),
               std::invalid_argument);
  ActuatorDiscreteConfig invalid = makeConfig();
  invalid.linear_delay_sec = 4.0;
  EXPECT_THROW((SyntheticModel(invalid, makePlantParams())),
               std::invalid_argument);
  ZohPlantParams invalid_plant = makePlantParams();
  invalid_plant.liquid.damping_ratio = -1.0;
  EXPECT_THROW((SyntheticModel(makeConfig(), invalid_plant)),
               std::invalid_argument);
}

TEST(MainlineActuatorDiscreteModel, MatchesIndependentCompleteMapGolden) {
  const SyntheticModel model(makeConfig(), makePlantParams());
  const SyntheticState state = makeState();
  const IssueControl control{4.0, 0.0, 0.7};
  SyntheticResult output;
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(state, control, output));

  EXPECT_DOUBLE_EQ(5.0, output.issued.linear_command);
  EXPECT_DOUBLE_EQ(-0.5, output.issued.angular_command);
  EXPECT_DOUBLE_EQ(6.0, output.issued.linear_acceleration);
  EXPECT_DOUBLE_EQ(0.5, output.issued.angular_acceleration);
  const std::array<double, 3> expected_duration{{0.25, 0.25, 0.5}};
  const std::array<double, 3> expected_linear{{1.0, 1.0, 5.0}};
  const std::array<double, 3> expected_angular{{-1.0, -0.5, -0.5}};
  for (std::size_t slot = 0; slot < output.segments.size(); ++slot) {
    EXPECT_DOUBLE_EQ(expected_duration[slot],
                     output.segments[slot].duration_sec);
    EXPECT_DOUBLE_EQ(expected_linear[slot],
                     output.segments[slot].linear_target);
    EXPECT_DOUBLE_EQ(expected_angular[slot],
                     output.segments[slot].angular_target);
  }

  EXPECT_NEAR(0.44866850507437234, output.next_state.physical.pose.x,
              1e-14);
  EXPECT_NEAR(-0.02201289323447934, output.next_state.physical.pose.y,
              1e-14);
  EXPECT_NEAR(-0.0823279219013971,
              output.next_state.physical.pose.heading, 1e-14);
  EXPECT_NEAR(1.2782662080017468,
              output.next_state.physical.actual.linear_velocity, 1e-14);
  EXPECT_NEAR(-0.13571377601879525,
              output.next_state.physical.actual.angular_velocity, 1e-14);
  EXPECT_NEAR(-0.39274896068327525,
              output.next_state.physical.liquid.eta_x, 1e-14);
  EXPECT_NEAR(-1.0229532415494726,
              output.next_state.physical.liquid.eta_x_dot, 1e-14);
  EXPECT_NEAR(0.008365546025445678,
              output.next_state.physical.liquid.eta_y, 1e-14);
  EXPECT_NEAR(0.03722246763348247,
              output.next_state.physical.liquid.eta_y_dot, 1e-14);
  EXPECT_NEAR(10.7, output.next_state.progress, 1e-14);
  EXPECT_DOUBLE_EQ(5.0,
                   output.next_state.publisher.previous_linear_command);
  EXPECT_DOUBLE_EQ(-0.5,
                   output.next_state.publisher.previous_angular_command);
  EXPECT_DOUBLE_EQ(6.0,
                   output.next_state.publisher.previous_linear_acceleration);
  EXPECT_DOUBLE_EQ(0.5,
                   output.next_state.publisher.previous_angular_acceleration);
  EXPECT_EQ((std::array<double, 2>{{1.0, 3.0}}),
            output.next_state.linear_older);
  EXPECT_EQ((std::array<double, 2>{{-1.0, 2.0}}),
            output.next_state.angular_older);
  expectStateDoubleEq(makeState(), state);
}

TEST(MainlineActuatorDiscreteModel, CurrentIssueCannotLeakBeforeOneStepDelay) {
  ActuatorDiscreteConfig config = makeConfig();
  config.maximum_linear_delay_sec = 1.0;
  config.maximum_angular_delay_sec = 1.0;
  config.linear_delay_sec = 1.0;
  config.angular_delay_sec = 1.0;
  const ActuatorDiscreteModel<2, 2> model(config, makePlantParams());
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
  ActuatorDiscreteConfig config = makeConfig();
  config.linear_delay_sec = 3.0;
  config.angular_delay_sec = 3.0;
  const SyntheticModel model(config, makePlantParams());
  const SyntheticState state = makeState();

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
  ActuatorDiscreteConfig config = makeConfig();
  config.maximum_linear_delay_sec = 0.0;
  config.maximum_angular_delay_sec = 0.0;
  config.linear_delay_sec = 0.0;
  config.angular_delay_sec = 0.0;
  const ActuatorDiscreteModel<1, 1> model(config, makePlantParams());
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
  ActuatorDiscreteConfig config = makeConfig();
  config.maximum_linear_delay_sec = 0.0;
  config.linear_delay_sec = 0.0;
  config.angular_delay_sec = 2.5;
  const ActuatorDiscreteModel<1, 4> model(config, makePlantParams());
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
  const SyntheticModel model(makeConfig(), makePlantParams());
  SyntheticResult first;
  ASSERT_EQ(ActuatorDiscreteStepStatus::kOk,
            model.step(makeState(), IssueControl{4.0, 0.0, 0.7}, first));
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
  const SyntheticModel model(makeConfig(), makePlantParams());
  const SyntheticResult sentinel = makeSentinel();
  SyntheticResult output = sentinel;
  SyntheticState state = makeState();

  state.linear_older[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ActuatorDiscreteStepStatus::kInvalidState,
            model.step(state, IssueControl{}, output));
  expectResultDoubleEq(sentinel, output);

  state = makeState();
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

  state = makeState();
  state.physical.actual = ActualMotionState{
      std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max()};
  EXPECT_EQ(ActuatorDiscreteStepStatus::kPlantPropagationFailure,
            model.step(state, IssueControl{}, output));
  expectResultDoubleEq(sentinel, output);

  state = makeState();
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
