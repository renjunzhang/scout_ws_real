#include "spmpc_local_planner/runtime/execution_prediction/execution_state_predictor.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

bool ExecutionStatePredictor::configure(const SloshModelParams& slosh_params) {
    SloshDynamics validator;
    slosh_configured_ = validator.configure(slosh_params);
    if (slosh_configured_) {
        slosh_params_ = slosh_params;
    }
    return slosh_configured_;
}

ExecutionStatePrediction ExecutionStatePredictor::predict(const RobotState& raw_robot,
                                                          const SloshState& raw_slosh,
                                                          const CommandHistoryBuffer& history,
                                                          StampNs now_ns,
                                                          const DelayPhaseParams& params) const {
    return predict(raw_robot, raw_slosh, history, now_ns, now_ns, params);
}

ExecutionStatePrediction ExecutionStatePredictor::predict(
    const RobotState& raw_robot,
    const SloshState& raw_slosh,
    const CommandHistoryBuffer& history,
    StampNs state_epoch_ns,
    StampNs evaluation_time_ns,
    const DelayPhaseParams& params) const {
    ExecutionStatePrediction out;
    out.raw_robot = raw_robot;
    out.raw_slosh = raw_slosh;
    out.predicted_robot = raw_robot;
    out.predicted_slosh = raw_slosh;
    out.linear_delay_sec = params.linear_delay_sec;
    out.angular_delay_sec = params.angular_delay_sec;
    out.linear_time_constant_sec = params.linear_time_constant_sec;
    out.angular_time_constant_sec = params.angular_time_constant_sec;
    out.prediction_origin_epoch_ns = validStamp(state_epoch_ns) ? state_epoch_ns : 0;
    out.history_span_sec = history.spanSec();

    const double state_age_sec = secondsBetween(evaluation_time_ns, state_epoch_ns);
    const bool valid_params = slosh_configured_ &&
        validStamp(state_epoch_ns) &&
        validStamp(evaluation_time_ns) && params.max_prediction_sec > 0.0 &&
        std::isfinite(state_age_sec) && state_age_sec >= 0.0 &&
        std::isfinite(params.linear_delay_sec) &&
        params.linear_delay_sec >= 0.0 &&
        std::isfinite(params.angular_delay_sec) &&
        params.angular_delay_sec >= 0.0 &&
        std::isfinite(params.linear_time_constant_sec) &&
        params.linear_time_constant_sec >= 0.0 &&
        std::isfinite(params.angular_time_constant_sec) &&
        params.angular_time_constant_sec >= 0.0 &&
        std::isfinite(params.max_integration_step_sec) &&
        params.max_integration_step_sec > 0.0 &&
        std::isfinite(params.min_integration_step_sec) &&
        params.min_integration_step_sec > 0.0;
    if (!valid_params) {
        out.status_code = DelayPhaseStatusCode::InvalidParams;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    ExecutionModelContract contract;
    contract.contract_id = "delay_phase_development_history_v1";
    contract.dt = slosh_params_.dt;
    contract.linear.delay_sec = params.linear_delay_sec;
    contract.linear.time_constant_sec = params.linear_time_constant_sec;
    contract.angular.delay_sec = params.angular_delay_sec;
    contract.angular.time_constant_sec = params.angular_time_constant_sec;
    ExecutionModel execution_model;
    std::string execution_model_error;
    if (!execution_model.configure(
            contract, slosh_params_, execution_model_error)) {
        out.status_code = DelayPhaseStatusCode::InvalidParams;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    const double execution_front_sec = execution_model.executionLeadSec();
    const double duration = state_age_sec + execution_front_sec;
    if (!std::isfinite(duration) ||
        duration > params.max_prediction_sec + 1e-9) {
        out.status_code = DelayPhaseStatusCode::InvalidParams;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }
    out.integrated_duration_sec = duration;
    out.prediction_epoch_ns = addSeconds(evaluation_time_ns, execution_front_sec);

    if (duration <= 1e-9) {
        out.history_complete = true;
        out.valid = true;
        out.status_code = delayPhaseReadyStatus(params.mode);
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    if (history.empty()) {
        out.missing_history_sec = duration;
        out.status_code = DelayPhaseStatusCode::NoCmdHistory;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    const double cmd_age = secondsBetween(evaluation_time_ns, history.latestStampNs());
    if (std::isfinite(params.cmd_timeout_sec) && params.cmd_timeout_sec > 0.0 &&
        std::isfinite(cmd_age) && cmd_age > params.cmd_timeout_sec) {
        out.status_code = DelayPhaseStatusCode::CmdStale;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    // The oldest command queried by either channel occurs at the physical
    // state epoch minus the largest pure delay.  The integration itself starts
    // at state_epoch and ends at evaluation_time + d_f; do not subtract the
    // integration duration here or the state age would be counted twice.
    const StampNs start_ns = addSeconds(
        state_epoch_ns, -execution_model.requiredHistorySec());
    const StampNs oldest_ns = history.oldestStampNs();
    if (validStamp(oldest_ns) && start_ns < oldest_ns) {
        out.missing_history_sec = std::min(
            duration, secondsBetween(oldest_ns, start_ns));
    }
    out.missing_history_sec = std::max(0.0, out.missing_history_sec);
    out.covered_history_sec = std::max(0.0, duration - out.missing_history_sec);
    out.history_complete = out.missing_history_sec <= 1e-6;

    if (params.require_complete_history && !out.history_complete) {
        out.status_code = DelayPhaseStatusCode::PartialHistory;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    const double max_step = std::max(params.min_integration_step_sec,
                                     std::min(params.max_integration_step_sec, duration));
    const double min_step = std::max(1e-6, std::min(params.min_integration_step_sec, max_step));
    if (!std::isfinite(max_step) || max_step <= 0.0) {
        out.status_code = DelayPhaseStatusCode::InvalidParams;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    const ExecutionHistoryRolloutResult rollout =
        execution_model.rolloutPublishedHistory(
            raw_robot, raw_slosh, history, state_epoch_ns,
            duration, max_step, min_step);
    if (!rollout.valid) {
        out.status_code = DelayPhaseStatusCode::InvalidParams;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    out.predicted_robot = rollout.robot;
    out.predicted_slosh = rollout.slosh;
    out.valid = true;
    if (out.history_complete) {
        out.status_code = delayPhaseReadyStatus(params.mode);
    } else {
        out.status_code = DelayPhaseStatusCode::PartialHistory;
    }
    out.status = delayPhaseStatusName(out.status_code);
    return out;
}

}  // namespace spmpc_local_planner
