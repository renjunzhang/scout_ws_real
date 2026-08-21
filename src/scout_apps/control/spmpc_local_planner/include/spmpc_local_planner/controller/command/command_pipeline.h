#pragma once

#include "spmpc_local_planner/domain/command.h"
#include "spmpc_local_planner/domain/time.h"

#include <string>

namespace spmpc_local_planner {

enum class CommandSource {
    None,
    Solver,
    Terminal,
    PhaseRejoin,
    Safety,
    FailClosed,
    ExecutionContract,
};

const char* commandSourceName(CommandSource source);

struct CommandCandidate {
    bool active = false;
    bool accepted = false;
    VelocityCommand command;
    std::string reason;
};

struct CommandArbitrationRequest {
    CommandCandidate solver;
    CommandCandidate terminal;
    CommandCandidate phase_rejoin;
    CommandCandidate terminal_reached;
    CommandCandidate safety;
};

struct CommandDecision {
    VelocityCommand command;
    CommandSource source = CommandSource::None;
    std::string reason = "NO_COMMAND";
    bool accepted = false;
};

CommandDecision arbitrateCommand(const CommandArbitrationRequest& request);

struct CommandPipelineConfig {
    double control_frequency = 30.0;
    bool linear_accel_limit_enable = true;
    double linear_accel_max = 0.6;
    double linear_accel_max_dt = 0.2;
    bool angular_limit_enable = false;
    double angular_rate_max = 1.2;
    double angular_accel_max = 1.2;
    double angular_accel_max_dt = 0.2;
    bool fail_closed_on_post_limit_change = false;
    double max_post_limit_delta_v = 1e-4;
    double max_post_limit_delta_omega = 1e-4;
};

struct CommandPipelineRequest {
    StampNs stamp_ns = 0;
    VelocityCommand desired;
    CommandSource source = CommandSource::Solver;
    std::string reason;
    bool force_zero = false;
    bool accepted = true;
};

struct CommandPipelineResult {
    CommandDecision decision;
    VelocityCommand desired;
    VelocityCommand previous;
    VelocityCommand final_command;
    double limiter_dt_sec = 0.0;
    bool linear_limited = false;
    bool angular_rate_limited = false;
    bool angular_accel_limited = false;
    bool finite_violation = false;
    bool command_contract_violation = false;
    bool command_was_published = false;
};

class CommandPipeline {
public:
    bool configure(const CommandPipelineConfig& config, std::string& error);
    void reset();

    CommandPipelineResult finalize(const CommandPipelineRequest& request);
    bool commitPublished(const VelocityCommand& command, StampNs stamp_ns);

    const CommandPipelineConfig& config() const { return config_; }
    bool hasPublishedCommand() const { return have_previous_; }
    const VelocityCommand& lastPublishedCommand() const { return previous_; }
    StampNs lastPublishStampNs() const { return previous_stamp_ns_; }

private:
    CommandPipelineConfig config_;
    VelocityCommand previous_;
    StampNs previous_stamp_ns_ = 0;
    bool have_previous_ = false;
};

}  // namespace spmpc_local_planner
