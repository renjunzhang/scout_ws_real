#pragma once

#include "spmpc_local_planner/controller/command/command_sink.h"
#include "spmpc_local_planner/runtime/execution_prediction/command_history_buffer.h"
#include "spmpc_local_planner/runtime/timing/publish_latency_model.h"

#include <cstdint>
#include <string>

namespace spmpc_local_planner {

// Optional shared execution disturbance/limiter stage.  It is applied inside
// the unique publication transaction after the ordinary command pipeline and
// before FinalCommand/sink publication, so receipt, limiter state and history
// all observe the same actually published command.
struct PrePublicationLinearCap {
    bool active = false;
    double max_linear = 0.0;
    std::string id;
};

struct CommandPublicationRequest {
    std::uint64_t cycle_id = 0;
    CommandDecision proposed;
    bool force_zero = false;
    bool publish_enabled = true;
    ICommandSink* sink = nullptr;
    CommandHistoryBuffer* history = nullptr;
    PrePublicationLinearCap linear_cap;
};

struct CommandPublicationResult {
    CommandPipelineResult pipeline;
    FinalCommand finalized;
    PublicationReceipt receipt;
    PublishLatencyObservation publish_timing;
    bool receipt_consistent = false;
    bool limiter_state_committed = false;
    bool history_committed = false;
    VelocityCommand pre_publication_stage_command;
    bool linear_cap_active = false;
    bool linear_cap_modified = false;
    std::string linear_cap_id;

    bool published() const {
        return receipt.delivered && receipt_consistent;
    }

    bool commandWasModified() const {
        return pipeline.linear_limited || pipeline.angular_rate_limited ||
            pipeline.angular_accel_limited ||
            pipeline.command_contract_violation || linear_cap_modified;
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
