#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace spmpc_local_planner {
namespace {

ExecutionModelContract baseContract() {
    ExecutionModelContract contract;
    contract.dt = 0.1;
    contract.linear.output_min = -2.0;
    contract.linear.output_max = 2.0;
    contract.angular.output_min = -3.0;
    contract.angular.output_max = 3.0;
    return contract;
}

SloshModelParams sloshParams() {
    SloshModelParams params;
    params.dt = 0.1;
    return params;
}

ExecutionModel configuredModel(const ExecutionModelContract& contract) {
    ExecutionModel model;
    std::string error;
    EXPECT_TRUE(model.configure(contract, sloshParams(), error)) << error;
    return model;
}

ExecutionAugmentedState heldState(const ExecutionModel& model) {
    ExecutionAugmentedState state;
    std::string error;
    EXPECT_TRUE(model.initializeHeld(
        RobotState{}, SloshState{}, VelocityCommand{}, state, error))
        << error;
    return state;
}

VelocityCommand command(double linear, double angular) {
    VelocityCommand value;
    value.linear = linear;
    value.angular = angular;
    return value;
}

void pushCommand(CommandHistoryBuffer& history,
                 StampNs stamp_ns,
                 double linear,
                 double angular) {
    TimedCommandSample sample;
    sample.stamp_ns = stamp_ns;
    sample.command = command(linear, angular);
    history.push(sample);
}

TEST(ExecutionModel, ResolvesPhysicalDelayIntoGridAndFractionalParts) {
    ExecutionModelContract contract = baseContract();
    contract.linear.delay_sec = 0.15;
    contract.angular.delay_sec = 0.20;
    ExecutionModel model = configuredModel(contract);

    EXPECT_EQ(1, model.contract().linear.integer_delay_steps);
    EXPECT_NEAR(0.05,
                model.contract().linear.fractional_delay_sec,
                1e-12);
    EXPECT_EQ(2, model.contract().angular.integer_delay_steps);
    EXPECT_DOUBLE_EQ(0.0,
                     model.contract().angular.fractional_delay_sec);
    EXPECT_NEAR(0.20, model.requiredHistorySec(), 1e-12);
    EXPECT_NEAR(0.20, model.executionLeadSec(), 1e-12);
    EXPECT_EQ(2, model.gridExecutionLeadSteps());
}

TEST(ExecutionModel, NewDecisionRespectsDifferentChannelDelays) {
    ExecutionModelContract contract = baseContract();
    contract.linear.delay_sec = 0.15;
    contract.angular.delay_sec = 0.25;
    ExecutionModel model = configuredModel(contract);
    ExecutionAugmentedState state = heldState(model);

    const ExecutionStepResult stage0 = model.step(state, command(1.0, 1.0));
    ASSERT_TRUE(stage0.valid);
    EXPECT_DOUBLE_EQ(0.0, stage0.state.robot.v);
    EXPECT_DOUBLE_EQ(0.0, stage0.state.robot.omega);

    const ExecutionStepResult stage1 = model.step(
        stage0.state, command(1.0, 1.0));
    ASSERT_TRUE(stage1.valid);
    ASSERT_EQ(2u, stage1.segments.size());
    EXPECT_DOUBLE_EQ(0.0, stage1.segments[0].target_v);
    EXPECT_DOUBLE_EQ(1.0, stage1.segments[1].target_v);
    EXPECT_DOUBLE_EQ(0.0, stage1.state.robot.omega);
    EXPECT_NEAR(0.05, stage1.state.robot.x, 1e-12);

    const ExecutionStepResult stage2 = model.step(
        stage1.state, command(1.0, 1.0));
    ASSERT_TRUE(stage2.valid);
    ASSERT_EQ(2u, stage2.segments.size());
    EXPECT_DOUBLE_EQ(1.0, stage2.segments[0].target_v);
    EXPECT_DOUBLE_EQ(0.0, stage2.segments[0].target_omega);
    EXPECT_DOUBLE_EQ(1.0, stage2.segments[1].target_omega);
    EXPECT_NEAR(1.0, stage2.state.robot.v, 1e-12);
    EXPECT_NEAR(1.0, stage2.state.robot.omega, 1e-12);
    EXPECT_NEAR(0.05, stage2.state.robot.yaw, 1e-12);
}

TEST(ExecutionModel, SplitsStageAtBothFractionalDelayEvents) {
    ExecutionModelContract contract = baseContract();
    contract.linear.delay_sec = 0.02;
    contract.angular.delay_sec = 0.07;
    ExecutionModel model = configuredModel(contract);

    const ExecutionStepResult result = model.step(
        heldState(model), command(1.0, 2.0));

    ASSERT_TRUE(result.valid);
    ASSERT_EQ(3u, result.segments.size());
    EXPECT_NEAR(0.02, result.segments[0].duration_sec, 1e-12);
    EXPECT_NEAR(0.05, result.segments[1].duration_sec, 1e-12);
    EXPECT_NEAR(0.03, result.segments[2].duration_sec, 1e-12);
    EXPECT_DOUBLE_EQ(0.0, result.segments[0].target_v);
    EXPECT_DOUBLE_EQ(0.0, result.segments[0].target_omega);
    EXPECT_DOUBLE_EQ(1.0, result.segments[1].target_v);
    EXPECT_DOUBLE_EQ(0.0, result.segments[1].target_omega);
    EXPECT_DOUBLE_EQ(1.0, result.segments[2].target_v);
    EXPECT_DOUBLE_EQ(2.0, result.segments[2].target_omega);
}

TEST(ExecutionModel, AppliesDeadzoneDirectionGainSaturationAndInertia) {
    ExecutionModelContract contract = baseContract();
    contract.linear.time_constant_sec = 0.2;
    contract.linear.positive_gain = 2.0;
    contract.linear.negative_gain = 1.0;
    contract.linear.deadzone = 0.1;
    contract.linear.output_min = -0.4;
    contract.linear.output_max = 0.5;
    ExecutionModel model = configuredModel(contract);

    const ExecutionStepResult positive = model.step(
        heldState(model), command(0.4, 0.0));
    ASSERT_TRUE(positive.valid);
    ASSERT_EQ(1u, positive.segments.size());
    EXPECT_DOUBLE_EQ(0.5, positive.segments[0].target_v);
    EXPECT_NEAR(0.5 * (1.0 - std::exp(-0.5)),
                positive.state.robot.v, 1e-12);

    ExecutionAugmentedState reset = heldState(model);
    const ExecutionStepResult deadzone = model.step(
        reset, command(0.1, 0.0));
    ASSERT_TRUE(deadzone.valid);
    EXPECT_DOUBLE_EQ(0.0, deadzone.segments[0].target_v);

    const ExecutionStepResult negative = model.step(
        reset, command(-0.3, 0.0));
    ASSERT_TRUE(negative.valid);
    EXPECT_NEAR(-0.2, negative.segments[0].target_v, 1e-12);
}

TEST(ExecutionModel, RejectsInvalidContractStateAndCommand) {
    ExecutionModel model;
    ExecutionModelContract contract = baseContract();
    contract.linear.delay_sec = -0.1;
    std::string error;
    EXPECT_FALSE(model.configure(contract, sloshParams(), error));
    EXPECT_FALSE(error.empty());

    contract = baseContract();
    ASSERT_TRUE(model.configure(contract, sloshParams(), error)) << error;
    ExecutionAugmentedState state = heldState(model);
    state.linear.pending_commands.clear();
    const ExecutionStepResult invalid_state = model.step(
        state, command(0.0, 0.0));
    EXPECT_FALSE(invalid_state.valid);
    EXPECT_EQ("INVALID_EXECUTION_STEP_INPUT", invalid_state.status);

    state = heldState(model);
    state.linear.actuator_output = 0.1;
    const ExecutionStepResult mismatched_actuator = model.step(
        state, command(0.0, 0.0));
    EXPECT_FALSE(mismatched_actuator.valid);

    state = heldState(model);
    const ExecutionStepResult invalid_command = model.step(
        state,
        command(std::numeric_limits<double>::quiet_NaN(), 0.0));
    EXPECT_FALSE(invalid_command.valid);
}

TEST(ExecutionModel,
     AlignsCompleteAugmentedStateFromRealPublishedHistory) {
    ExecutionModelContract contract = baseContract();
    contract.linear.delay_sec = 0.15;
    contract.angular.delay_sec = 0.25;
    ExecutionModel model = configuredModel(contract);

    CommandHistoryBuffer history;
    history.configure(2.0);
    pushCommand(history, 600000000LL, 0.1, 0.1);
    pushCommand(history, 700000000LL, 0.2, 0.2);
    pushCommand(history, 800000000LL, 0.3, 0.3);
    pushCommand(history, 900000000LL, 0.4, 0.4);

    RobotState robot;
    robot.v = 0.2;
    robot.omega = 0.1;
    const ExecutionAugmentedAlignmentResult result =
        model.alignPublishedHistory(
            robot, SloshState{}, history,
            850000000LL, 1000000000LL, 0.05, 0.001);

    ASSERT_TRUE(result.valid) << result.status;
    EXPECT_TRUE(result.history_complete);
    EXPECT_EQ(result.source_epoch_ns, 850000000LL);
    EXPECT_EQ(result.target_epoch_ns, 1000000000LL);
    EXPECT_NEAR(result.integrated_duration_sec, 0.15, 1e-12);
    EXPECT_NEAR(result.command_age_sec, 0.10, 1e-12);
    EXPECT_EQ(result.state.stage_index, 0u);
    ASSERT_EQ(result.state.linear.pending_commands.size(), 2u);
    EXPECT_DOUBLE_EQ(result.state.linear.pending_commands[0], 0.3);
    EXPECT_DOUBLE_EQ(result.state.linear.pending_commands[1], 0.4);
    ASSERT_EQ(result.state.angular.pending_commands.size(), 3u);
    EXPECT_DOUBLE_EQ(result.state.angular.pending_commands[0], 0.2);
    EXPECT_DOUBLE_EQ(result.state.angular.pending_commands[1], 0.3);
    EXPECT_DOUBLE_EQ(result.state.angular.pending_commands[2], 0.4);
    EXPECT_DOUBLE_EQ(result.state.robot.v, 0.3);
    EXPECT_DOUBLE_EQ(result.state.robot.omega, 0.2);
    EXPECT_DOUBLE_EQ(result.state.linear.actuator_output,
                     result.state.robot.v);
    EXPECT_DOUBLE_EQ(result.state.angular.actuator_output,
                     result.state.robot.omega);
    EXPECT_GT(std::abs(result.state.slosh.eta_x) +
              std::abs(result.state.slosh.eta_x_dot) +
              std::abs(result.state.slosh.eta_y) +
              std::abs(result.state.slosh.eta_y_dot),
              0.0);
    ASSERT_EQ(result.segments.size(), 3u);
}

TEST(ExecutionModel,
     AugmentedAlignmentResamplesIrregularHistoryByTimeSlot) {
    ExecutionModelContract contract = baseContract();
    contract.linear.delay_sec = 0.15;
    contract.angular.delay_sec = 0.25;
    ExecutionModel model = configuredModel(contract);

    CommandHistoryBuffer history;
    pushCommand(history, 600000000LL, 1.0, 10.0);
    pushCommand(history, 700000000LL, 2.0, 20.0);
    // The 0.8 s publication is deliberately missing.  The delay queue must
    // repeat the 0.7 s held command, not pull the 0.6 s event forward merely
    // to obtain the expected number of entries.
    pushCommand(history, 900000000LL, 5.0, 50.0);

    const ExecutionAugmentedAlignmentResult result =
        model.alignPublishedHistory(
            RobotState{}, SloshState{}, history,
            850000000LL, 1000000000LL, 0.05, 0.001);

    ASSERT_TRUE(result.valid) << result.status;
    EXPECT_TRUE(result.history_complete);
    ASSERT_EQ(result.state.linear.pending_commands.size(), 2u);
    EXPECT_DOUBLE_EQ(result.state.linear.pending_commands[0], 2.0);
    EXPECT_DOUBLE_EQ(result.state.linear.pending_commands[1], 5.0);
    ASSERT_EQ(result.state.angular.pending_commands.size(), 3u);
    EXPECT_DOUBLE_EQ(result.state.angular.pending_commands[0], 20.0);
    EXPECT_DOUBLE_EQ(result.state.angular.pending_commands[1], 20.0);
    EXPECT_DOUBLE_EQ(result.state.angular.pending_commands[2], 50.0);
}

TEST(ExecutionModel, AugmentedAlignmentRejectsMissingTimeCoverage) {
    ExecutionModelContract contract = baseContract();
    contract.linear.delay_sec = 0.15;
    contract.angular.delay_sec = 0.25;
    ExecutionModel model = configuredModel(contract);

    CommandHistoryBuffer history;
    pushCommand(history, 700000000LL, 2.0, 20.0);
    pushCommand(history, 900000000LL, 5.0, 50.0);

    const ExecutionAugmentedAlignmentResult result =
        model.alignPublishedHistory(
            RobotState{}, SloshState{}, history,
            850000000LL, 1000000000LL, 0.05, 0.001);

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.history_complete);
    EXPECT_EQ(result.status, "INCOMPLETE_PHYSICAL_COMMAND_HISTORY");
}

TEST(ExecutionModel, AugmentedAlignmentRejectsTargetEpochHistory) {
    ExecutionModelContract contract = baseContract();
    ExecutionModel model = configuredModel(contract);

    CommandHistoryBuffer history;
    pushCommand(history, 800000000LL, 1.0, 2.0);
    pushCommand(history, 1000000000LL, 3.0, 4.0);

    const ExecutionAugmentedAlignmentResult result =
        model.alignPublishedHistory(
            RobotState{}, SloshState{}, history,
            900000000LL, 1000000000LL, 0.05, 0.001);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status, "COMMAND_HISTORY_NOT_BEFORE_TARGET");
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
