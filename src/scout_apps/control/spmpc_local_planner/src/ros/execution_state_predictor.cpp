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
    ExecutionStatePrediction out;
    out.raw_robot = raw_robot;
    out.raw_slosh = raw_slosh;
    out.predicted_robot = raw_robot;
    out.predicted_slosh = raw_slosh;
    out.linear_delay_sec = params.linear_delay_sec;
    out.angular_delay_sec = params.angular_delay_sec;
    out.history_span_sec = history.spanSec();

    if (now.isZero() || params.max_prediction_sec <= 0.0) {
        out.status_code = DelayPhaseStatusCode::InvalidParams;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    double duration = std::max(0.0, std::max(params.linear_delay_sec, params.angular_delay_sec));
    duration = std::min(duration, std::max(0.0, params.max_prediction_sec));
    if (!std::isfinite(duration) || duration <= 0.0) {
        out.status_code = DelayPhaseStatusCode::InvalidParams;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }
    out.integrated_duration_sec = duration;

    if (history.empty()) {
        out.missing_history_sec = duration;
        out.status_code = DelayPhaseStatusCode::NoCmdHistory;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    const double cmd_age = (now - history.latestStamp()).toSec();
    if (std::isfinite(params.cmd_timeout_sec) && params.cmd_timeout_sec > 0.0 &&
        std::isfinite(cmd_age) && cmd_age > params.cmd_timeout_sec) {
        out.status_code = DelayPhaseStatusCode::CmdStale;
        out.status = delayPhaseStatusName(out.status_code);
        return out;
    }

    const ros::Time start = now - ros::Duration(duration);
    const ros::Time oldest = history.oldestStamp();
    if (!oldest.isZero() && start < oldest) {
        out.missing_history_sec = std::min(duration, (oldest - start).toSec());
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
    ros::Time t = start;
    while (elapsed < duration - 1e-9) {
        double step = std::min(max_step, duration - elapsed);
        if (step < min_step && duration - elapsed > min_step) {
            step = min_step;
        }

        TimedCommandSample command_sample;
        geometry_msgs::Twist cmd;
        if (history.sampleAt(t, command_sample)) {
            cmd = command_sample.cmd;
        }

        const double v = cmd.linear.x;
        const double omega = cmd.angular.z;
        robot.x += v * std::cos(robot.yaw) * step;
        robot.y += v * std::sin(robot.yaw) * step;
        robot.yaw = normalizeYaw(robot.yaw + omega * step);
        robot.v = v;
        robot.omega = omega;

        if (slosh_configured_) {
            SloshDynamics step_model = slosh_dynamics_;
            if (std::abs(step - step_model.params().dt) > 1e-6) {
                auto slosh_params = step_model.params();
                slosh_params.dt = step;
                step_model.configure(slosh_params);
            }
            const double ax = (v - prev_v) / std::max(1e-6, step);
            const double ay = v * omega;
            slosh = step_model.step(slosh, ax, ay, omega);
        }
        prev_v = v;

        elapsed += step;
        t += ros::Duration(step);
    }

    out.predicted_robot = robot;
    out.predicted_slosh = slosh;
    out.valid = true;
    if (out.history_complete) {
        out.status_code = params.mode == DelayPhaseMode::FixedClosedLoop
                              ? DelayPhaseStatusCode::FixedClosedLoopOk
                              : DelayPhaseStatusCode::ShadowOk;
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
