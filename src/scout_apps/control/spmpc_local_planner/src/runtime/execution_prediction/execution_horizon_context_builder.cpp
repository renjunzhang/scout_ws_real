#include "spmpc_local_planner/runtime/execution_prediction/execution_horizon_context_builder.h"

#include <cmath>
#include <limits>

namespace spmpc_local_planner {

bool ExecutionHorizonContextBuilder::configure(
    const ExecutionModelContract& contract,
    const SloshModelParams& slosh_params,
    const ExecutionHorizonBuilderConfig& config,
    std::string& error) {
    configured_ = false;
    error.clear();
    if (contract.contract_id.empty() || contract.contract_hash.empty()) {
        error = "formal execution contract id/hash must be non-empty";
        return false;
    }
    if (!std::isfinite(config.command_timeout_sec) ||
        config.command_timeout_sec <= 0.0 ||
        !std::isfinite(config.max_alignment_sec) ||
        config.max_alignment_sec <= 0.0 ||
        !std::isfinite(config.max_integration_step_sec) ||
        config.max_integration_step_sec <= 0.0 ||
        !std::isfinite(config.min_integration_step_sec) ||
        config.min_integration_step_sec <= 0.0 ||
        config.min_integration_step_sec >
            config.max_integration_step_sec) {
        error = "invalid execution horizon builder limits";
        return false;
    }
    if (!execution_model_.configure(contract, slosh_params, error)) {
        return false;
    }
    config_ = config;
    configured_ = true;
    return true;
}

ExecutionHorizonBuildResult ExecutionHorizonContextBuilder::build(
    const ExecutionHorizonBuildRequest& request) const {
    ExecutionHorizonBuildResult result;
    if (!configured_) {
        result.status = "EXECUTION_HORIZON_BUILDER_NOT_CONFIGURED";
        return result;
    }
    if (request.expected_execution_contract_hash.empty() ||
        request.expected_execution_contract_hash !=
            execution_model_.contract().contract_hash) {
        result.status = "EXECUTION_CONTRACT_HASH_MISMATCH";
        return result;
    }
    if (!publishEpochEstimateMatchesCycle(
            request.publish_epoch_estimate,
            request.publish_epoch_estimate.cycle) ||
        !request.publish_epoch_estimate.valid) {
        result.status = "INVALID_PUBLISH_EPOCH_ESTIMATE";
        return result;
    }
    if (request.publish_epoch_estimate.expected_deadline_missed) {
        result.status = "EXPECTED_PUBLISH_DEADLINE_MISSED";
        return result;
    }
    if (std::abs(
            request.publish_epoch_estimate.cycle.control_period_sec -
            execution_model_.contract().dt) > 1e-12) {
        result.status = "EXECUTION_CONTROL_PERIOD_MISMATCH";
        return result;
    }
    if (!validStamp(request.source_epoch_ns) ||
        request.publish_epoch_estimate.expected_publish_stamp_ns <
            request.source_epoch_ns) {
        result.status = "EXPECTED_PUBLISH_BEFORE_SOURCE_EPOCH";
        return result;
    }
    if (!std::isfinite(request.initial_progress_s) ||
        request.initial_progress_s < 0.0 ||
        request.liquid_horizon_steps <= 0) {
        result.status = "INVALID_EXECUTION_HORIZON_REQUEST";
        return result;
    }
    if (!request.command_history) {
        result.status = "NO_COMMAND_HISTORY_PORT";
        return result;
    }

    const StampNs target_epoch_ns =
        request.publish_epoch_estimate.expected_publish_stamp_ns;
    const double alignment_sec = secondsBetween(
        target_epoch_ns, request.source_epoch_ns);
    if (!std::isfinite(alignment_sec) || alignment_sec < 0.0 ||
        alignment_sec > config_.max_alignment_sec + 1e-12) {
        result.status = "EXECUTION_ALIGNMENT_DURATION_EXCEEDED";
        return result;
    }
    if (request.command_history->empty()) {
        result.status = "COMMAND_HISTORY_EMPTY";
        return result;
    }
    const double command_age_sec = secondsBetween(
        target_epoch_ns, request.command_history->latestStampNs());
    if (!std::isfinite(command_age_sec) || command_age_sec < 0.0 ||
        command_age_sec > config_.command_timeout_sec) {
        result.status = "COMMAND_HISTORY_STALE";
        return result;
    }

    result.alignment = execution_model_.alignPublishedHistory(
        request.source_robot,
        request.source_slosh,
        *request.command_history,
        request.source_epoch_ns,
        target_epoch_ns,
        config_.max_integration_step_sec,
        config_.min_integration_step_sec);
    if (!result.alignment.valid ||
        !result.alignment.history_complete) {
        result.status = "EXECUTION_ALIGNMENT_" +
            result.alignment.status;
        return result;
    }

    const int execution_front_steps =
        execution_model_.gridExecutionLeadSteps();
    if (execution_front_steps < 0 ||
        execution_front_steps >
            std::numeric_limits<int>::max() -
                request.liquid_horizon_steps) {
        result.status = "EXECUTION_HORIZON_CARDINALITY_OVERFLOW";
        return result;
    }
    const int horizon_steps = execution_front_steps +
        request.liquid_horizon_steps;

    result.context.active = true;
    result.context.contract = execution_model_.contract();
    result.context.initial_state = result.alignment.state;
    result.context.initial_progress_s = request.initial_progress_s;
    result.context.initial_epoch_ns = target_epoch_ns;
    result.context.execution_front_steps = execution_front_steps;
    result.context.liquid_horizon_steps =
        request.liquid_horizon_steps;
    result.context.horizon_steps = horizon_steps;
    result.context.physical_front_epoch_ns = addSeconds(
        target_epoch_ns, execution_model_.executionLeadSec());
    result.context.grid_front_epoch_ns = addSeconds(
        target_epoch_ns,
        static_cast<double>(execution_front_steps) *
            execution_model_.contract().dt);
    result.context.terminal_epoch_ns = addSeconds(
        target_epoch_ns,
        static_cast<double>(horizon_steps) *
            execution_model_.contract().dt);
    if (!validStamp(result.context.physical_front_epoch_ns) ||
        !validStamp(result.context.grid_front_epoch_ns) ||
        !validStamp(result.context.terminal_epoch_ns)) {
        result.context = ExecutionHorizonContext{};
        result.status = "EXECUTION_HORIZON_EPOCH_OVERFLOW";
        return result;
    }

    result.valid = true;
    result.status = "OK";
    return result;
}

}  // namespace spmpc_local_planner
