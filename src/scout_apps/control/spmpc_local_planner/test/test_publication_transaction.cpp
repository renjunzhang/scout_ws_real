#include "spmpc_local_planner/controller/command/publication_transaction.h"

#include <gtest/gtest.h>

#include <limits>

namespace spmpc_local_planner {
namespace {

class FakeSink : public ICommandSink {
public:
    StampNs publicationTimeNs() override {
        return now_ns;
    }

    PublicationReceipt publish(const FinalCommand& command) override {
        ++publish_calls;
        last = command;
        PublicationReceipt receipt;
        receipt.cycle_id = command.cycle_id;
        receipt.attempted = true;
        receipt.command = command.command;
        if (!command.publish_enabled) {
            receipt.status = "PUBLISH_DISABLED";
            return receipt;
        }
        if (!deliver) {
            receipt.status = "FAKE_FAILURE";
            return receipt;
        }
        receipt.delivered = true;
        receipt.actual_publish_stamp_ns = now_ns;
        receipt.status = "FAKE_DELIVERED";
        if (mutate_receipt) {
            receipt.command.linear += 0.25;
        }
        return receipt;
    }

    StampNs now_ns = secondsToNanoseconds(1.0);
    bool deliver = true;
    bool mutate_receipt = false;
    int publish_calls = 0;
    FinalCommand last;
};

CommandPipeline makePipeline(bool limit = false) {
    CommandPipelineConfig config;
    config.control_frequency = 10.0;
    config.linear_accel_limit_enable = limit;
    config.linear_accel_max = 1.0;
    config.linear_accel_max_dt = 0.2;
    config.angular_limit_enable = false;
    CommandPipeline pipeline;
    std::string error;
    EXPECT_TRUE(pipeline.configure(config, error)) << error;
    return pipeline;
}

CommandPublicationRequest requestFor(
    FakeSink& sink,
    CommandHistoryBuffer& history) {
    CommandPublicationRequest request;
    request.cycle_id = 42;
    request.proposed.command.linear = 0.8;
    request.proposed.command.angular = -0.2;
    request.proposed.source = CommandSource::Solver;
    request.proposed.reason = "OK";
    request.proposed.accepted = true;
    request.sink = &sink;
    request.history = &history;
    return request;
}

TEST(PublicationTransaction, PublishesOnceAndCommitsOneCommandTruth) {
    CommandPipeline pipeline = makePipeline(true);
    PublicationTransaction transaction(pipeline);
    FakeSink sink;
    CommandHistoryBuffer history;
    history.configure(2.0);

    const CommandPublicationResult result = transaction.execute(
        requestFor(sink, history));

    EXPECT_EQ(1, sink.publish_calls);
    EXPECT_TRUE(result.published());
    EXPECT_TRUE(result.limiter_state_committed);
    EXPECT_TRUE(result.history_committed);
    EXPECT_TRUE(result.pipeline.linear_limited);
    EXPECT_DOUBLE_EQ(0.1, result.finalized.command.linear);
    EXPECT_DOUBLE_EQ(result.finalized.command.linear,
                     result.receipt.command.linear);
    EXPECT_DOUBLE_EQ(result.receipt.command.linear,
                     pipeline.lastPublishedCommand().linear);

    TimedCommandSample sample;
    ASSERT_TRUE(history.sampleAt(history.latestStampNs(), sample));
    EXPECT_DOUBLE_EQ(result.receipt.command.linear, sample.command.linear);
    EXPECT_DOUBLE_EQ(result.receipt.command.angular, sample.command.angular);
    EXPECT_TRUE(sample.meta.linear_limited);
}

TEST(PublicationTransaction, FailedReceiptDoesNotAdvanceHistoryOrLimiter) {
    CommandPipeline pipeline = makePipeline();
    PublicationTransaction transaction(pipeline);
    FakeSink sink;
    sink.deliver = false;
    CommandHistoryBuffer history;

    const CommandPublicationResult result = transaction.execute(
        requestFor(sink, history));

    EXPECT_EQ(1, sink.publish_calls);
    EXPECT_FALSE(result.published());
    EXPECT_FALSE(result.history_committed);
    EXPECT_FALSE(result.limiter_state_committed);
    EXPECT_TRUE(history.empty());
    EXPECT_FALSE(pipeline.hasPublishedCommand());
}

TEST(PublicationTransaction, InconsistentReceiptUsesActualHistoryButBlocksCommit) {
    CommandPipeline pipeline = makePipeline();
    PublicationTransaction transaction(pipeline);
    FakeSink sink;
    sink.mutate_receipt = true;
    CommandHistoryBuffer history;

    const CommandPublicationResult result = transaction.execute(
        requestFor(sink, history));

    EXPECT_EQ(1, sink.publish_calls);
    EXPECT_TRUE(result.receipt.delivered);
    EXPECT_FALSE(result.receipt_consistent);
    EXPECT_FALSE(result.published());
    EXPECT_TRUE(result.history_committed);
    EXPECT_DOUBLE_EQ(result.receipt.command.linear,
                     pipeline.lastPublishedCommand().linear);
    TimedCommandSample sample;
    ASSERT_TRUE(history.sampleAt(history.latestStampNs(), sample));
    EXPECT_DOUBLE_EQ(result.receipt.command.linear, sample.command.linear);
}

TEST(PublicationTransaction, DisabledPublicationStillUsesSingleSinkBoundary) {
    CommandPipeline pipeline = makePipeline();
    PublicationTransaction transaction(pipeline);
    FakeSink sink;
    CommandHistoryBuffer history;
    CommandPublicationRequest request = requestFor(sink, history);
    request.publish_enabled = false;

    const CommandPublicationResult result = transaction.execute(request);

    EXPECT_EQ(1, sink.publish_calls);
    EXPECT_TRUE(result.receipt.attempted);
    EXPECT_FALSE(result.receipt.delivered);
    EXPECT_EQ("PUBLISH_DISABLED", result.receipt.status);
    EXPECT_TRUE(history.empty());
    EXPECT_FALSE(pipeline.hasPublishedCommand());
}

TEST(PublicationTransaction,
     LinearCapPublishesAndCommitsExactlyOnePostCapCommandTruth) {
    CommandPipeline pipeline = makePipeline();
    PublicationTransaction transaction(pipeline);
    FakeSink sink;
    CommandHistoryBuffer history;
    history.configure(2.0);
    CommandPublicationRequest request = requestFor(sink, history);
    request.linear_cap.active = true;
    request.linear_cap.max_linear = 0.32;
    request.linear_cap.id = "D2_SHORT_LINEAR_SPEED_CAP";

    const CommandPublicationResult result = transaction.execute(request);

    EXPECT_EQ(1, sink.publish_calls);
    EXPECT_TRUE(result.published());
    EXPECT_TRUE(result.linear_cap_active);
    EXPECT_TRUE(result.linear_cap_modified);
    EXPECT_TRUE(result.commandWasModified());
    EXPECT_TRUE(result.limiter_state_committed);
    EXPECT_TRUE(result.history_committed);
    EXPECT_EQ("D2_SHORT_LINEAR_SPEED_CAP", result.linear_cap_id);
    EXPECT_DOUBLE_EQ(0.8, result.pre_publication_stage_command.linear);
    EXPECT_DOUBLE_EQ(-0.2, result.pre_publication_stage_command.angular);
    EXPECT_DOUBLE_EQ(0.32, result.pipeline.final_command.linear);
    EXPECT_DOUBLE_EQ(-0.2, result.pipeline.final_command.angular);
    EXPECT_DOUBLE_EQ(result.pipeline.final_command.linear,
                     result.pipeline.decision.command.linear);
    EXPECT_DOUBLE_EQ(result.pipeline.final_command.linear,
                     result.finalized.command.linear);
    EXPECT_DOUBLE_EQ(result.finalized.command.linear,
                     result.receipt.command.linear);
    EXPECT_DOUBLE_EQ(result.receipt.command.linear,
                     pipeline.lastPublishedCommand().linear);

    TimedCommandSample sample;
    ASSERT_TRUE(history.sampleAt(history.latestStampNs(), sample));
    EXPECT_DOUBLE_EQ(result.receipt.command.linear, sample.command.linear);
    EXPECT_DOUBLE_EQ(result.receipt.command.angular, sample.command.angular);
}

TEST(PublicationTransaction, LooseLinearCapIsAnAuditedNoOp) {
    CommandPipeline pipeline = makePipeline();
    PublicationTransaction transaction(pipeline);
    FakeSink sink;
    CommandHistoryBuffer history;
    CommandPublicationRequest request = requestFor(sink, history);
    request.linear_cap.active = true;
    request.linear_cap.max_linear = 0.9;
    request.linear_cap.id = "LOOSE_CAP";

    const CommandPublicationResult result = transaction.execute(request);

    EXPECT_EQ(1, sink.publish_calls);
    EXPECT_TRUE(result.published());
    EXPECT_TRUE(result.linear_cap_active);
    EXPECT_FALSE(result.linear_cap_modified);
    EXPECT_FALSE(result.commandWasModified());
    EXPECT_DOUBLE_EQ(0.8, result.finalized.command.linear);
    EXPECT_DOUBLE_EQ(-0.2, result.finalized.command.angular);
}

TEST(PublicationTransaction, NonfiniteLinearCapFailsClosedOnce) {
    CommandPipeline pipeline = makePipeline();
    PublicationTransaction transaction(pipeline);
    FakeSink sink;
    CommandHistoryBuffer history;
    CommandPublicationRequest request = requestFor(sink, history);
    request.linear_cap.active = true;
    request.linear_cap.max_linear =
        std::numeric_limits<double>::quiet_NaN();
    request.linear_cap.id = "INVALID_CAP";

    const CommandPublicationResult result = transaction.execute(request);

    EXPECT_EQ(1, sink.publish_calls);
    EXPECT_TRUE(result.published());
    EXPECT_TRUE(result.pipeline.command_contract_violation);
    EXPECT_FALSE(result.pipeline.decision.accepted);
    EXPECT_EQ(CommandSource::ExecutionContract,
              result.pipeline.decision.source);
    EXPECT_EQ("PRE_PUBLICATION_LINEAR_CAP_INVALID",
              result.pipeline.decision.reason);
    EXPECT_DOUBLE_EQ(0.0, result.finalized.command.linear);
    EXPECT_DOUBLE_EQ(0.0, result.finalized.command.angular);
    EXPECT_DOUBLE_EQ(result.finalized.command.linear,
                     result.receipt.command.linear);
    EXPECT_DOUBLE_EQ(result.receipt.command.linear,
                     pipeline.lastPublishedCommand().linear);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
