#include "spmpc_local_planner/runtime/execution_prediction/execution_horizon_context_builder.h"
#include "spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h"

#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

ExecutionModelContract executionContract() {
    ExecutionModelContract contract;
    contract.contract_id = "formal_alignment_test_v1";
    contract.contract_hash = "formal-alignment-test-hash";
    contract.dt = 0.1;
    contract.linear.delay_sec = 0.15;
    contract.angular.delay_sec = 0.25;
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

void pushCommand(CommandHistoryBuffer& history,
                 StampNs stamp_ns,
                 double value) {
    TimedCommandSample sample;
    sample.stamp_ns = stamp_ns;
    sample.command.linear = value;
    sample.command.angular = value;
    history.push(sample);
}

CommandHistoryBuffer completeHistory() {
    CommandHistoryBuffer history;
    history.configure(2.0);
    pushCommand(history, 600000000LL, 0.1);
    pushCommand(history, 700000000LL, 0.2);
    pushCommand(history, 800000000LL, 0.3);
    pushCommand(history, 900000000LL, 0.4);
    return history;
}

PublishEpochEstimate publishEstimate() {
    PublishLatencyModel model;
    PublishLatencyModelConfig config;
    config.enabled = true;
    config.estimated_dc_sec = 0.05;
    std::string error;
    EXPECT_TRUE(model.configure(config, error)) << error;
    CycleTimingContract cycle;
    cycle.cycle_id = 11;
    cycle.cycle_start_stamp_ns = 950000000LL;
    cycle.control_period_sec = 0.1;
    return model.estimate(cycle);
}

ExecutionHorizonContextBuilder configuredBuilder() {
    ExecutionHorizonContextBuilder builder;
    ExecutionHorizonBuilderConfig config;
    config.command_timeout_sec = 0.2;
    config.max_alignment_sec = 0.5;
    config.max_integration_step_sec = 0.05;
    config.min_integration_step_sec = 0.001;
    std::string error;
    EXPECT_TRUE(builder.configure(
        executionContract(), sloshParams(), config, error)) << error;
    return builder;
}

ExecutionHorizonBuildRequest validRequest(
    const CommandHistoryBuffer& history) {
    ExecutionHorizonBuildRequest request;
    request.source_robot.v = 0.2;
    request.source_robot.omega = 0.1;
    request.source_epoch_ns = 850000000LL;
    request.publish_epoch_estimate = publishEstimate();
    request.command_history = &history;
    request.expected_execution_contract_hash =
        executionContract().contract_hash;
    request.initial_progress_s = 1.25;
    request.liquid_horizon_steps = 2;
    return request;
}

}  // namespace

TEST(ExecutionHorizonContextBuilder,
     BuildsExpectedPublishEpochAugmentedContext) {
    const CommandHistoryBuffer history = completeHistory();
    const ExecutionHorizonContextBuilder builder = configuredBuilder();

    const ExecutionHorizonBuildResult result = builder.build(
        validRequest(history));

    ASSERT_TRUE(result.valid) << result.status;
    ASSERT_TRUE(result.context.active);
    EXPECT_TRUE(result.alignment.history_complete);
    EXPECT_EQ(result.context.initial_epoch_ns, 1000000000LL);
    EXPECT_EQ(result.context.execution_front_steps, 3);
    EXPECT_EQ(result.context.liquid_horizon_steps, 2);
    EXPECT_EQ(result.context.horizon_steps, 5);
    EXPECT_EQ(result.context.physical_front_epoch_ns, 1250000000LL);
    EXPECT_EQ(result.context.grid_front_epoch_ns, 1300000000LL);
    EXPECT_EQ(result.context.terminal_epoch_ns, 1500000000LL);
    EXPECT_DOUBLE_EQ(result.context.initial_progress_s, 1.25);
    EXPECT_EQ(result.context.contract.contract_hash,
              executionContract().contract_hash);
    ASSERT_EQ(
        result.context.initial_state.linear.pending_commands.size(), 2u);
    ASSERT_EQ(
        result.context.initial_state.angular.pending_commands.size(), 3u);
    EXPECT_DOUBLE_EQ(
        result.context.initial_state.linear.pending_commands.back(), 0.4);
    EXPECT_DOUBLE_EQ(
        result.context.initial_state.angular.pending_commands.front(), 0.2);
}

TEST(ExecutionHorizonContextBuilder, RejectsContractHashMutation) {
    const CommandHistoryBuffer history = completeHistory();
    const ExecutionHorizonContextBuilder builder = configuredBuilder();
    ExecutionHorizonBuildRequest request = validRequest(history);
    request.expected_execution_contract_hash += "-mutated";

    const ExecutionHorizonBuildResult result = builder.build(request);

    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.context.active);
    EXPECT_EQ(result.status, "EXECUTION_CONTRACT_HASH_MISMATCH");
}

TEST(ExecutionHorizonContextBuilder, RejectsDisabledPublishEstimate) {
    const CommandHistoryBuffer history = completeHistory();
    const ExecutionHorizonContextBuilder builder = configuredBuilder();
    ExecutionHorizonBuildRequest request = validRequest(history);
    request.publish_epoch_estimate.valid = false;
    request.publish_epoch_estimate.expected_publish_stamp_ns = 0;
    request.publish_epoch_estimate.status = "ESTIMATE_OFF";

    const ExecutionHorizonBuildResult result = builder.build(request);

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status, "INVALID_PUBLISH_EPOCH_ESTIMATE");
}

TEST(ExecutionHorizonContextBuilder, RejectsIncompleteHistory) {
    CommandHistoryBuffer history;
    pushCommand(history, 600000000LL, 0.1);
    pushCommand(history, 900000000LL, 0.4);
    const ExecutionHorizonContextBuilder builder = configuredBuilder();

    const ExecutionHorizonBuildResult result = builder.build(
        validRequest(history));

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status,
              "EXECUTION_ALIGNMENT_INCOMPLETE_PENDING_COMMAND_HISTORY");
}

TEST(ExecutionHorizonContextBuilder, RejectsStaleHistory) {
    CommandHistoryBuffer history;
    pushCommand(history, 400000000LL, 0.1);
    pushCommand(history, 500000000LL, 0.2);
    pushCommand(history, 600000000LL, 0.3);
    pushCommand(history, 700000000LL, 0.4);
    const ExecutionHorizonContextBuilder builder = configuredBuilder();

    const ExecutionHorizonBuildResult result = builder.build(
        validRequest(history));

    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status, "COMMAND_HISTORY_STALE");
}

TEST(ExecutionHorizonContextBuilder,
     OutputPassesSolverValidatorAndMutationsFailClosed) {
    const CommandHistoryBuffer history = completeHistory();
    const ExecutionHorizonContextBuilder builder = configuredBuilder();
    const ExecutionHorizonBuildResult result = builder.build(
        validRequest(history));
    ASSERT_TRUE(result.valid) << result.status;

    DelayAugmentedPhaseDynamics dynamics;
    std::string error;
    ASSERT_TRUE(dynamics.configure(
        executionContract(), sloshParams(), error)) << error;
    EXPECT_TRUE(dynamics.validateHorizonContext(
        result.context, error)) << error;

    ExecutionHorizonContext mutated = result.context;
    mutated.contract.contract_hash += "-mutated";
    EXPECT_FALSE(dynamics.validateHorizonContext(mutated, error));

    mutated = result.context;
    ++mutated.horizon_steps;
    EXPECT_FALSE(dynamics.validateHorizonContext(mutated, error));

    mutated = result.context;
    ++mutated.terminal_epoch_ns;
    EXPECT_FALSE(dynamics.validateHorizonContext(mutated, error));
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
