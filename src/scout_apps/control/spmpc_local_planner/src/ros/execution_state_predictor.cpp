#include "spmpc_local_planner/ros/execution_state_predictor.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

bool ExecutionStatePredictor::configure(const SloshModelParams& slosh_params) {
    slosh_configured_ = slosh_dynamics_.configure(slosh_params);
    return slosh_configured_;
}

ExecutionStatePrediction ExecutionStatePredictor::predict(const RobotState& raw_robot,
                                                          const SloshState& raw_slosh,
                                                          const CommandHistoryBuffer& history,
                                                          const ros::Time& now,
                                                          const DelayPhaseParams& params) const {
    return predict(raw_robot, raw_slosh, history, now, now, params);
}

ExecutionStatePrediction ExecutionStatePredictor::predict(
    const RobotState& raw_robot,
    const SloshState& raw_slosh,
    const CommandHistoryBuffer& history,
    const ros::Time& state_epoch,
    const ros::Time& evaluation_time,
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
    out.prediction_origin_epoch_ns = state_epoch.isZero()
        ? 0
        : static_cast<std::int64_t>(state_epoch.toNSec());
    out.history_span_sec = history.spanSec();

    const double state_age_sec = (evaluation_time - state_epoch).toSec();
    const bool valid_params = !state_epoch.isZero() &&
        !evaluation_time.isZero() && params.max_prediction_sec > 0.0 &&
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

    const double execution_front_sec = std::max(
        0.0, std::max(params.linear_delay_sec, params.angular_delay_sec));
    const double duration = state_age_sec + execution_front_sec;
    if (!std::isfinite(duration) ||
        duration > params.max_prediction_sec + 1e-9) {
        out.status_code = DelayPhaseStatusCode::InvalidParams;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }
    out.integrated_duration_sec = duration;
    const ros::Time prediction_epoch =
        evaluation_time + ros::Duration(execution_front_sec);
    out.prediction_epoch_ns = static_cast<std::int64_t>(
        prediction_epoch.toNSec());

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

    const double cmd_age = (evaluation_time - history.latestStamp()).toSec();
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
    const ros::Time start =
        state_epoch - ros::Duration(execution_front_sec);
    const ros::Time oldest = history.oldestStamp();
    if (!oldest.isZero() && start < oldest) {
        out.missing_history_sec = std::min(
            duration, (oldest - start).toSec());
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

    RobotState robot = raw_robot;
    SloshState slosh = raw_slosh;
    double prev_v = raw_robot.v;
    double elapsed = 0.0;
    while (elapsed < duration - 1e-9) {
        double step = std::min(max_step, duration - elapsed);
        if (step < min_step && duration - elapsed > min_step) {
            step = min_step;
        }

        const ros::Time future_time = state_epoch + ros::Duration(elapsed);
        TimedCommandSample linear_sample;
        TimedCommandSample angular_sample;
        double target_v = 0.0;
        double target_omega = 0.0;
        if (history.sampleAt(
                future_time - ros::Duration(params.linear_delay_sec),
                linear_sample)) {
            target_v = linear_sample.cmd.linear.x;
        }
        if (history.sampleAt(
                future_time - ros::Duration(params.angular_delay_sec),
                angular_sample)) {
            target_omega = angular_sample.cmd.angular.z;
        }

        const double linear_gain = params.linear_time_constant_sec <= 1e-9
            ? 1.0
            : 1.0 - std::exp(-step / params.linear_time_constant_sec);
        const double angular_gain = params.angular_time_constant_sec <= 1e-9
            ? 1.0
            : 1.0 - std::exp(-step / params.angular_time_constant_sec);
        const double v = robot.v + linear_gain * (target_v - robot.v);
        const double omega = robot.omega +
            angular_gain * (target_omega - robot.omega);
        robot.x += v * std::cos(robot.yaw) * step;
        robot.y += v * std::sin(robot.yaw) * step;
        robot.yaw = normalizeYaw(robot.yaw + omega * step);
        robot.v = v;
        robot.omega = omega;

        if (slosh_configured_) {
            const double ax = (v - prev_v) / std::max(1e-6, step);
            const double ay = v * omega;
            SloshState next_slosh;
            if (!slosh_dynamics_.stepWithDt(
                    slosh, ax, ay, omega, step, next_slosh)) {
                out.status_code = DelayPhaseStatusCode::InvalidParams;
                out.status = delayPhaseStatusName(out.status_code);
                return out;
            }
            slosh = next_slosh;
        }
        prev_v = v;

        elapsed += step;
    }

    out.predicted_robot = robot;
    out.predicted_slosh = slosh;
    out.valid = true;
    if (out.history_complete) {
        out.status_code = delayPhaseReadyStatus(params.mode);
    } else {
        out.status_code = DelayPhaseStatusCode::PartialHistory;
    }
    out.status = delayPhaseStatusName(out.status_code);
    return out;
}

double ExecutionStatePredictor::normalizeYaw(double yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

}  // namespace spmpc_local_planner
