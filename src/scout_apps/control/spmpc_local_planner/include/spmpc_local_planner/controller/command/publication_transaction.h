#pragma once

#include "spmpc_local_planner/controller/command/command_sink.h"
#include "spmpc_local_planner/runtime/execution_prediction/command_history_buffer.h"

#include <cstdint>

namespace spmpc_local_planner {

struct CommandPublicationRequest {
    std::uint64_t cycle_id = 0;
    CommandDecision proposed;
    bool force_zero = false;
    bool publish_enabled = true;
    ICommandSink* sink = nullptr;
    CommandHistoryBuffer* history = nullptr;
};

struct CommandPublicationResult {
    CommandPipelineResult pipeline;
    FinalCommand finalized;
    PublicationReceipt receipt;
    bool receipt_consistent = false;
    bool limiter_state_committed = false;
    bool history_committed = false;

    bool published() const {
        return receipt.delivered && receipt_consistent;
    }

    bool commandWasModified() const {
        return pipeline.linear_limited || pipeline.angular_rate_limited ||
            pipeline.angular_accel_limited ||
            pipeline.command_contract_violation;
    }
};

class PublicationTransaction {
public:
    explicit PublicationTransaction(CommandPipeline& pipeline)
        : pipeline_(pipeline) {}

    CommandPublicationResult execute(
        const CommandPublicationRequest& request);

private:
    CommandPipeline& pipeline_;
};

}  // namespace spmpc_local_planner
