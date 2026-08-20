#include "spmpc_local_planner/controller/command/command_pipeline.h"

#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

CommandCandidate candidate(double linear, const char* reason) {
    CommandCandidate out;
    out.active = true;
    out.accepted = true;
    out.command.linear = linear;
    out.reason = reason;
    return out;
}

CommandPipeline configuredPipeline(bool fail_closed = false) {
    CommandPipelineConfig config;
    config.control_frequency = 10.0;
    config.linear_accel_limit_enable = true;
    config.linear_accel_max = 1.0;
    config.linear_accel_max_dt = 0.2;
    config.angular_limit_enable = true;
    config.angular_rate_max = 0.5;
    config.angular_accel_max = 1.0;
    config.angular_accel_max_dt = 0.2;
    config.fail_closed_on_post_limit_change = fail_closed;
    config.max_post_limit_delta_v = 1e-4;
    config.max_post_limit_delta_omega = 1e-4;
    CommandPipeline pipeline;
    std::string error;
    EXPECT_TRUE(pipeline.configure(config, error)) << error;
    return pipeline;
}

TEST(CommandPriority, SafetyOverridesEveryCandidate) {
    CommandArbitrationRequest request;
    request.solver = candidate(1.0, "solver");
    request.terminal = candidate(2.0, "terminal");
    request.phase_rejoin = candidate(3.0, "phase");
    request.terminal_reached = candidate(4.0, "reached");
    request.safety = candidate(0.0, "unsafe");

    const auto decision = arbitrateCommand(request);
    EXPECT_EQ(decision.source, CommandSource::Safety);
    EXPECT_DOUBLE_EQ(decision.command.linear, 0.0);
    EXPECT_EQ(decision.reason, "unsafe");
}

TEST(CommandPriority, TerminalReachedThenPhaseThenEnvelopeThenSolver) {
    CommandArbitrationRequest request;
    request.solver = candidate(1.0, "solver");
    request.terminal = candidate(2.0, "terminal");
    request.phase_rejoin = candidate(3.0, "phase");
    request.terminal_reached = candidate(0.0, "reached");

    EXPECT_EQ(arbitrateCommand(request).source, CommandSource::Terminal);
    request.terminal_reached.active = false;
    EXPECT_EQ(arbitrateCommand(request).source, CommandSource::PhaseRejoin);
    request.phase_rejoin.active = false;
    EXPECT_EQ(arbitrateCommand(request).source, CommandSource::Terminal);
    request.terminal.active = false;
    EXPECT_EQ(arbitrateCommand(request).source, CommandSource::Solver);
    request.solver.active = false;
    EXPECT_EQ(arbitrateCommand(request).source, CommandSource::FailClosed);
}

TEST(CommandPipeline, PreservesHistoricalFirstAndSubsequentRateLimitSemantics) {
    auto pipeline = configuredPipeline();
    CommandPipelineRequest request;
    request.stamp_ns = secondsToNanoseconds(1.0);
    request.desired.linear = 1.0;
    request.desired.angular = 1.0;

    const auto first = pipeline.finalize(request);
    EXPECT_NEAR(first.limiter_dt_sec, 0.1, 1e-12);
    EXPECT_NEAR(first.final_command.linear, 0.1, 1e-12);
    EXPECT_NEAR(first.final_command.angular, 0.1, 1e-12);
    EXPECT_TRUE(first.linear_limited);
    EXPECT_TRUE(first.angular_rate_limited);
    EXPECT_TRUE(first.angular_accel_limited);

    request.stamp_ns = secondsToNanoseconds(1.2);
    const auto second = pipeline.finalize(request);
    EXPECT_NEAR(second.limiter_dt_sec, 0.2, 1e-12);
    EXPECT_NEAR(second.previous.linear, 0.1, 1e-12);
    EXPECT_NEAR(second.final_command.linear, 0.3, 1e-12);
    EXPECT_NEAR(second.final_command.angular, 0.3, 1e-12);
}

TEST(CommandPipeline, ForceZeroBypassesRateLimiterAndBecomesHistory) {
    auto pipeline = configuredPipeline();
    CommandPipelineRequest moving;
    moving.stamp_ns = secondsToNanoseconds(1.0);
    moving.desired.linear = 1.0;
    pipeline.finalize(moving);

    CommandPipelineRequest stop;
    stop.stamp_ns = secondsToNanoseconds(1.1);
    stop.force_zero = true;
    const auto result = pipeline.finalize(stop);
    EXPECT_DOUBLE_EQ(result.final_command.linear, 0.0);
    EXPECT_FALSE(result.linear_limited);
    EXPECT_EQ(result.decision.source, CommandSource::FailClosed);
    EXPECT_DOUBLE_EQ(pipeline.lastPublishedCommand().linear, 0.0);
}

TEST(CommandPipeline, ExecutionContractFailsClosedAfterLimiterChange) {
    auto pipeline = configuredPipeline(true);
    CommandPipelineRequest request;
    request.stamp_ns = secondsToNanoseconds(1.0);
    request.desired.linear = 1.0;
    const auto result = pipeline.finalize(request);

    EXPECT_TRUE(result.command_contract_violation);
    EXPECT_DOUBLE_EQ(result.final_command.linear, 0.0);
    EXPECT_EQ(result.decision.source, CommandSource::ExecutionContract);
    EXPECT_EQ(result.decision.reason,
              "COMMAND_EXECUTION_CONTRACT_VIOLATION");
}

TEST(CommandPipeline, DisabledPublicationDoesNotAdvanceLimiterState) {
    auto pipeline = configuredPipeline();
    CommandPipelineRequest disabled;
    disabled.stamp_ns = secondsToNanoseconds(1.0);
    disabled.desired.linear = 1.0;
    disabled.publish_enabled = false;
    const auto skipped = pipeline.finalize(disabled);
    EXPECT_FALSE(skipped.command_was_published);
    EXPECT_FALSE(pipeline.hasPublishedCommand());

    disabled.publish_enabled = true;
    disabled.stamp_ns = secondsToNanoseconds(5.0);
    const auto first_real = pipeline.finalize(disabled);
    EXPECT_NEAR(first_real.limiter_dt_sec, 0.1, 1e-12);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
