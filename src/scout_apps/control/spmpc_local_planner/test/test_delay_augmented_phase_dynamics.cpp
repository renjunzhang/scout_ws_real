#include "spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace spmpc_local_planner {
namespace {

ExecutionModelContract contract() {
    ExecutionModelContract value;
    value.contract_id = "test_delay_augmented_v1";
    value.contract_hash = "test-hash";
    value.dt = 0.1;
    value.linear.delay_sec = 0.15;
    value.linear.output_min = -2.0;
    value.linear.output_max = 2.0;
    value.angular.delay_sec = 0.25;
    value.angular.output_min = -3.0;
    value.angular.output_max = 3.0;
    return value;
}

SloshModelParams sloshParams() {
    SloshModelParams value;
    value.dt = 0.1;
    return value;
}

DelayAugmentedPhaseDynamics configuredDynamics() {
    DelayAugmentedPhaseDynamics dynamics;
    std::string error;
    EXPECT_TRUE(dynamics.configure(contract(), sloshParams(), error))
        << error;
    return dynamics;
}

DelayAugmentedPhaseState heldState(
    const DelayAugmentedPhaseDynamics& dynamics,
    double measured_v = 0.0,
    double measured_omega = 0.0,
    double held_v = 0.0,
    double held_omega = 0.0) {
    RobotState robot;
    robot.v = measured_v;
    robot.omega = measured_omega;
    VelocityCommand held;
    held.linear = held_v;
    held.angular = held_omega;
    DelayAugmentedPhaseState state;
    std::string error;
    EXPECT_TRUE(dynamics.initializeHeld(
        robot, SloshState{}, held, 1.25, state, error)) << error;
    return state;
}

ExecutionHorizonContext contextFor(
    const DelayAugmentedPhaseDynamics& dynamics,
    const DelayAugmentedPhaseState& state,
    int liquid_steps = 2) {
    ExecutionHorizonContext context;
    std::string error;
    EXPECT_TRUE(dynamics.makeHorizonContext(
        state, secondsToNanoseconds(10.0), liquid_steps,
        context, error)) << error;
    return context;
}

DelayAugmentedPhaseControl control(
    double acceleration,
    double angular_acceleration,
    double progress_rate = 0.0) {
    DelayAugmentedPhaseControl value;
    value.acceleration = acceleration;
    value.angular_acceleration = angular_acceleration;
    value.progress_rate = progress_rate;
    return value;
}

TEST(DelayAugmentedPhaseDynamics,
     BuildsPhysicalGridAndTerminalEpochsFromOneContract) {
    const DelayAugmentedPhaseDynamics dynamics = configuredDynamics();
    const ExecutionHorizonContext context = contextFor(
        dynamics, heldState(dynamics));

    EXPECT_TRUE(context.active);
    EXPECT_EQ(3, context.execution_front_steps);
    EXPECT_EQ(2, context.liquid_horizon_steps);
    EXPECT_EQ(5, context.horizon_steps);
    EXPECT_EQ(secondsToNanoseconds(10.25),
              context.physical_front_epoch_ns);
    EXPECT_EQ(secondsToNanoseconds(10.30),
              context.grid_front_epoch_ns);
    EXPECT_EQ(secondsToNanoseconds(10.50),
              context.terminal_epoch_ns);
    EXPECT_EQ(2u, context.initial_state.linear.pending_commands.size());
    EXPECT_EQ(3u, context.initial_state.angular.pending_commands.size());
}

TEST(DelayAugmentedPhaseDynamics,
     CurrentDecisionIntegratesFromLastPublishedCommandNotMeasuredVelocity) {
    const DelayAugmentedPhaseDynamics dynamics = configuredDynamics();
    const DelayAugmentedPhaseState initial = heldState(
        dynamics, 0.7, -0.4, 0.2, -0.1);

    const DelayAugmentedPhaseStepResult result = dynamics.step(
        initial, control(0.5, -0.5, 0.3));

    ASSERT_TRUE(result.valid) << result.status;
    EXPECT_NEAR(0.25, result.published_command.linear, 1e-12);
    EXPECT_NEAR(-0.15, result.published_command.angular, 1e-12);
    EXPECT_NEAR(1.28, result.state.progress_s, 1e-12);
    EXPECT_NEAR(0.25,
                result.state.execution.linear.pending_commands.back(),
                1e-12);
    EXPECT_NEAR(-0.15,
                result.state.execution.angular.pending_commands.back(),
                1e-12);
}

TEST(DelayAugmentedPhaseDynamics,
     FirstDecisionRespectsDistinctFractionalChannelDelays) {
    const DelayAugmentedPhaseDynamics dynamics = configuredDynamics();
    DelayAugmentedPhaseState state = heldState(dynamics);

    const DelayAugmentedPhaseStepResult stage0 = dynamics.step(
        state, control(1.0, 1.0));
    ASSERT_TRUE(stage0.valid);
    EXPECT_DOUBLE_EQ(0.0, stage0.state.execution.robot.v);
    EXPECT_DOUBLE_EQ(0.0, stage0.state.execution.robot.omega);

    const DelayAugmentedPhaseStepResult stage1 = dynamics.step(
        stage0.state, control(0.0, 0.0));
    ASSERT_TRUE(stage1.valid);
    EXPECT_NEAR(0.1, stage1.state.execution.robot.v, 1e-12);
    EXPECT_DOUBLE_EQ(0.0, stage1.state.execution.robot.omega);
    EXPECT_NEAR(0.005, stage1.state.execution.robot.x, 1e-12);

    const DelayAugmentedPhaseStepResult stage2 = dynamics.step(
        stage1.state, control(0.0, 0.0));
    ASSERT_TRUE(stage2.valid);
    EXPECT_NEAR(0.1, stage2.state.execution.robot.v, 1e-12);
    EXPECT_NEAR(0.1, stage2.state.execution.robot.omega, 1e-12);
    EXPECT_NEAR(0.005, stage2.state.execution.robot.yaw, 1e-12);
}

TEST(DelayAugmentedPhaseDynamics,
     FormalHorizonIsExecutionFrontPlusLiquidWindow) {
    const DelayAugmentedPhaseDynamics dynamics = configuredDynamics();
    const ExecutionHorizonContext context = contextFor(
        dynamics, heldState(dynamics, 0.2, 0.0, 0.2, 0.0), 2);
    std::vector<DelayAugmentedPhaseControl> controls(
        static_cast<std::size_t>(context.horizon_steps));
    controls[0] = control(0.4, -0.3, 0.2);

    const DelayAugmentedPhaseRolloutResult result = dynamics.rollout(
        context, controls);

    ASSERT_TRUE(result.valid) << result.status;
    EXPECT_EQ(5, result.horizon_steps);
    EXPECT_EQ(6u, result.states.size());
    EXPECT_EQ(5u, result.controls.size());
    EXPECT_EQ(5u, result.published_commands.size());
    EXPECT_NE(result.states.back().execution.robot.x,
              result.states.front().execution.robot.x);
    EXPECT_NE(result.states.back().execution.slosh.eta_x,
              result.states.front().execution.slosh.eta_x);
}

TEST(DelayAugmentedPhaseDynamics,
     FirstDecisionHasNonzeroJointTerminalSensitivityAfterItsDelays) {
    const DelayAugmentedPhaseDynamics dynamics = configuredDynamics();
    const ExecutionHorizonContext context = contextFor(
        dynamics, heldState(dynamics, 0.2, 0.0, 0.2, 0.0), 2);
    std::vector<DelayAugmentedPhaseControl> positive(
        static_cast<std::size_t>(context.horizon_steps));
    std::vector<DelayAugmentedPhaseControl> negative = positive;
    positive[0] = control(1e-3, 1e-3);
    negative[0] = control(-1e-3, -1e-3);

    const auto plus = dynamics.rollout(context, positive);
    const auto minus = dynamics.rollout(context, negative);

    ASSERT_TRUE(plus.valid);
    ASSERT_TRUE(minus.valid);
    const RobotState& plus_robot = plus.states.back().execution.robot;
    const RobotState& minus_robot = minus.states.back().execution.robot;
    const SloshState& plus_slosh = plus.states.back().execution.slosh;
    const SloshState& minus_slosh = minus.states.back().execution.slosh;
    EXPECT_GT(std::abs(plus_robot.x - minus_robot.x), 1e-9);
    EXPECT_GT(std::abs(plus_robot.yaw - minus_robot.yaw), 1e-9);
    EXPECT_GT(std::abs(plus_slosh.eta_x - minus_slosh.eta_x), 1e-12);
    EXPECT_GT(std::abs(plus_slosh.eta_y - minus_slosh.eta_y), 1e-12);
}

TEST(DelayAugmentedPhaseDynamics,
     RejectsContextMutationWrongCardinalityAndNonfiniteControl) {
    const DelayAugmentedPhaseDynamics dynamics = configuredDynamics();
    ExecutionHorizonContext context = contextFor(
        dynamics, heldState(dynamics));
    std::string error;

    ++context.terminal_epoch_ns;
    EXPECT_FALSE(dynamics.validateHorizonContext(context, error));
    EXPECT_FALSE(error.empty());

    context = contextFor(dynamics, heldState(dynamics));
    std::vector<DelayAugmentedPhaseControl> short_controls(4);
    const auto short_rollout = dynamics.rollout(context, short_controls);
    EXPECT_FALSE(short_rollout.valid);
    EXPECT_EQ("CONTROL_HORIZON_CARDINALITY_MISMATCH",
              short_rollout.status);

    DelayAugmentedPhaseControl invalid;
    invalid.acceleration = std::numeric_limits<double>::quiet_NaN();
    const auto invalid_step = dynamics.step(heldState(dynamics), invalid);
    EXPECT_FALSE(invalid_step.valid);
    EXPECT_EQ("INVALID_DELAY_AUGMENTED_STEP_INPUT", invalid_step.status);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
