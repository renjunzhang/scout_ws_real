#include "spmpc_local_planner/controller/command/command_pipeline.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

CommandDecision selectCandidate(const CommandCandidate& candidate,
                                CommandSource source,
                                const char* fallback_reason) {
    CommandDecision decision;
    decision.command = candidate.command;
    decision.source = source;
    decision.reason = candidate.reason.empty() ? fallback_reason : candidate.reason;
    decision.accepted = candidate.accepted;
    return decision;
}

}  // namespace

const char* commandSourceName(CommandSource source) {
    switch (source) {
    case CommandSource::Solver:
        return "SOLVER";
    case CommandSource::Terminal:
        return "TERMINAL";
    case CommandSource::PhaseRejoin:
        return "PHASE_REJOIN";
    case CommandSource::Safety:
        return "SAFETY";
    case CommandSource::FailClosed:
        return "FAIL_CLOSED";
    case CommandSource::ExecutionContract:
        return "EXECUTION_CONTRACT";
    case CommandSource::None:
    default:
        return "NONE";
    }
}

CommandDecision arbitrateCommand(const CommandArbitrationRequest& request) {
    if (request.safety.active) {
        return selectCandidate(request.safety, CommandSource::Safety,
                               "SAFETY_OVERRIDE");
    }
    if (request.terminal_reached.active) {
        return selectCandidate(request.terminal_reached, CommandSource::Terminal,
                               "GOAL_REACHED_LATCH");
    }
    if (request.phase_rejoin.active) {
        return selectCandidate(request.phase_rejoin, CommandSource::PhaseRejoin,
                               "PHASE_REJOIN_OVERRIDE");
    }
    if (request.terminal.active) {
        return selectCandidate(request.terminal, CommandSource::Terminal,
                               "TERMINAL_OVERRIDE");
    }
    if (request.solver.active) {
        return selectCandidate(request.solver, CommandSource::Solver,
                               "SOLVER_COMMAND");
    }

    CommandDecision decision;
    decision.source = CommandSource::FailClosed;
    decision.reason = "NO_ACCEPTED_COMMAND";
    return decision;
}

bool CommandPipeline::configure(const CommandPipelineConfig& config,
                                std::string& error) {
    error.clear();
    const bool valid = std::isfinite(config.control_frequency) &&
        config.control_frequency > 0.0 &&
        std::isfinite(config.linear_accel_max) && config.linear_accel_max >= 0.0 &&
        std::isfinite(config.linear_accel_max_dt) && config.linear_accel_max_dt > 0.0 &&
        std::isfinite(config.angular_rate_max) && config.angular_rate_max >= 0.0 &&
        std::isfinite(config.angular_accel_max) && config.angular_accel_max >= 0.0 &&
        std::isfinite(config.angular_accel_max_dt) && config.angular_accel_max_dt > 0.0 &&
        std::isfinite(config.max_post_limit_delta_v) &&
        config.max_post_limit_delta_v >= 0.0 &&
        std::isfinite(config.max_post_limit_delta_omega) &&
        config.max_post_limit_delta_omega >= 0.0;
    if (!valid) {
        error = "invalid command pipeline limits or execution contract";
        return false;
    }
    config_ = config;
    reset();
    return true;
}

void CommandPipeline::reset() {
    previous_ = VelocityCommand{};
    previous_stamp_ns_ = 0;
    have_previous_ = false;
}

CommandPipelineResult CommandPipeline::finalize(
    const CommandPipelineRequest& request) {
    CommandPipelineResult result;
    result.desired = request.desired;
    result.previous = previous_;
    result.final_command = request.force_zero
        ? VelocityCommand{}
        : request.desired;

    result.finite_violation =
        !std::isfinite(result.final_command.linear) ||
        !std::isfinite(result.final_command.angular);
    if (result.finite_violation) {
        result.final_command = VelocityCommand{};
        result.decision.source = CommandSource::ExecutionContract;
        result.decision.reason = "COMMAND_NONFINITE";
        result.decision.accepted = false;
    }

    const double nominal_dt = 1.0 / config_.control_frequency;
    result.limiter_dt_sec = nominal_dt;
    if (have_previous_ && validStamp(previous_stamp_ns_) &&
        validStamp(request.stamp_ns)) {
        result.limiter_dt_sec = secondsBetween(
            request.stamp_ns, previous_stamp_ns_);
    }
    if (!std::isfinite(result.limiter_dt_sec) ||
        result.limiter_dt_sec <= 1e-6) {
        result.limiter_dt_sec = nominal_dt;
    }

    if (!request.force_zero && !result.finite_violation) {
        const double linear_dt = std::min(
            result.limiter_dt_sec, config_.linear_accel_max_dt);
        const double angular_dt = std::min(
            result.limiter_dt_sec, config_.angular_accel_max_dt);

        if (config_.linear_accel_limit_enable &&
            config_.linear_accel_max > 0.0) {
            const double max_step = config_.linear_accel_max * linear_dt;
            result.final_command.linear = previous_.linear + clamp(
                request.desired.linear - previous_.linear, -max_step, max_step);
            result.linear_limited =
                std::abs(result.final_command.linear - request.desired.linear) > 1e-6;
        }

        if (config_.angular_limit_enable) {
            if (config_.angular_rate_max > 0.0) {
                const double before = result.final_command.angular;
                result.final_command.angular = clamp(
                    result.final_command.angular,
                    -config_.angular_rate_max,
                    config_.angular_rate_max);
                result.angular_rate_limited =
                    std::abs(result.final_command.angular - before) > 1e-6;
            }
            if (config_.angular_accel_max > 0.0) {
                const double before = result.final_command.angular;
                const double max_step = config_.angular_accel_max * angular_dt;
                result.final_command.angular = previous_.angular + clamp(
                    result.final_command.angular - previous_.angular,
                    -max_step, max_step);
                result.angular_accel_limited =
                    std::abs(result.final_command.angular - before) > 1e-6;
            }
        }

        result.command_contract_violation =
            std::abs(result.final_command.linear - request.desired.linear) >
                config_.max_post_limit_delta_v ||
            std::abs(result.final_command.angular - request.desired.angular) >
                config_.max_post_limit_delta_omega;
        if (result.command_contract_violation &&
            config_.fail_closed_on_post_limit_change) {
            result.final_command = VelocityCommand{};
            result.decision.source = CommandSource::ExecutionContract;
            result.decision.reason = "COMMAND_EXECUTION_CONTRACT_VIOLATION";
            result.decision.accepted = false;
        }
    }

    if (result.decision.source != CommandSource::ExecutionContract) {
        result.decision.source = request.source == CommandSource::None
            ? CommandSource::FailClosed
            : request.source;
        result.decision.reason = request.reason.empty()
            ? commandSourceName(result.decision.source)
            : request.reason;
        result.decision.accepted = request.accepted && !request.force_zero;
    }
    result.decision.command = result.final_command;
    return result;
}

bool CommandPipeline::commitPublished(
    const VelocityCommand& command,
    StampNs stamp_ns) {
    if (!validStamp(stamp_ns) || !std::isfinite(command.linear) ||
        !std::isfinite(command.angular)) {
        return false;
    }
    previous_ = command;
    previous_stamp_ns_ = stamp_ns;
    have_previous_ = true;
    return true;
}

}  // namespace spmpc_local_planner
