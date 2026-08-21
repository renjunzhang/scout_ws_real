#pragma once

#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"
#include "spmpc_local_planner/runtime/timing/publish_latency_model.h"
#include "spmpc_local_planner/solver/api/execution_horizon_context.h"

#include <string>

namespace spmpc_local_planner {

struct ExecutionHorizonBuilderConfig {
    double command_timeout_sec = 0.5;
    double max_alignment_sec = 0.5;
    double max_integration_step_sec = 0.02;
    double min_integration_step_sec = 0.001;
};

struct ExecutionHorizonBuildRequest {
    RobotState source_robot;
    SloshState source_slosh;
    StampNs source_epoch_ns = 0;
    PublishEpochEstimate publish_epoch_estimate;
    const CommandHistoryBuffer* command_history = nullptr;
    std::string expected_execution_contract_hash;
    double initial_progress_s = 0.0;
    int liquid_horizon_steps = 0;
};

struct ExecutionHorizonBuildResult {
    bool valid = false;
    std::string status = "NOT_EVALUATED";
    ExecutionHorizonContext context;
    ExecutionAugmentedAlignmentResult alignment;
};

// ROS-independent formal input builder.  configure() freezes one resolved
// execution contract; every cycle must explicitly bind the same hash and a
// complete PublishEpochEstimate image before an active context can be made.
class ExecutionHorizonContextBuilder {
public:
    bool configure(const ExecutionModelContract& contract,
                   const SloshModelParams& slosh_params,
                   const ExecutionHorizonBuilderConfig& config,
                   std::string& error);

    ExecutionHorizonBuildResult build(
        const ExecutionHorizonBuildRequest& request) const;

    bool configured() const { return configured_; }
    const ExecutionModelContract& contract() const {
        return execution_model_.contract();
    }

private:
    ExecutionModel execution_model_;
    ExecutionHorizonBuilderConfig config_;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
