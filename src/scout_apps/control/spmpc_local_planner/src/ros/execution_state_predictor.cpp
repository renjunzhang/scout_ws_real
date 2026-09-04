#include "spmpc_local_planner/ros/execution_state_predictor.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

bool ExecutionStatePredictor::configure(
    const SloshModelParams& slosh_params,
    double explicit_prefix_step_sec) {
    slosh_configured_ = slosh_dynamics_.configure(slosh_params);
    explicit_prefix_slosh_configured_ = false;
    if (slosh_configured_ && std::isfinite(explicit_prefix_step_sec) &&
        explicit_prefix_step_sec > 1e-9) {
        SloshModelParams prefix_params = slosh_params;
        prefix_params.dt = explicit_prefix_step_sec;
        explicit_prefix_slosh_configured_ =
            explicit_prefix_slosh_dynamics_.configure(prefix_params);
    }
    return slosh_configured_ &&
           (explicit_prefix_step_sec <= 1e-9 ||
            explicit_prefix_slosh_configured_);
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
        out.status_code = delayPhaseReadyStatus(params.mode);
    } else {
        out.status_code = DelayPhaseStatusCode::PartialHistory;
    }
    out.status = delayPhaseStatusName(out.status_code);
    return out;
}

ExplicitActuatorPrediction ExecutionStatePredictor::predictExplicitActuator(
    const RobotState& raw_robot,
    const SloshState& raw_slosh,
    const CommandHistoryBuffer& history,
    const ros::Time& state_epoch,
    const ros::Time& target_epoch,
    const ActuatorModelParams& params) const {
    ExplicitActuatorPrediction out;
    out.raw_robot = raw_robot;
    out.raw_slosh = raw_slosh;
    out.predicted_robot = raw_robot;
    out.predicted_slosh = raw_slosh;
    out.history_span_sec = history.spanSec();

    std::string params_error;
    if (params.mode != ExecutionModelMode::ExplicitActuator ||
        !validateActuatorModelParams(params, &params_error)) {
        out.status = "INVALID_ACTUATOR_PARAMS:" + params_error;
        return out;
    }
    if (state_epoch.isZero() || target_epoch.isZero() ||
        target_epoch < state_epoch) {
        out.status = "INVALID_ACTUATOR_EPOCH";
        return out;
    }
    out.prefix_duration_sec = (target_epoch - state_epoch).toSec();
    if (!std::isfinite(out.prefix_duration_sec) ||
        out.prefix_duration_sec > params.max_prefix_prediction_sec + 1e-9) {
        out.status = "ACTUATOR_PREFIX_TOO_LONG";
        return out;
    }
    if (history.empty()) {
        out.status = "NO_CMD_HISTORY";
        return out;
    }
    const double cmd_age = (target_epoch - history.latestStamp()).toSec();
    if (!std::isfinite(cmd_age) || cmd_age < -1e-6 ||
        (params.cmd_timeout_sec > 0.0 && cmd_age > params.cmd_timeout_sec)) {
        out.status = "CMD_HISTORY_STALE";
        return out;
    }

    bool complete = true;
    const auto sampleCommand = [&](const ros::Time& stamp,
                                   geometry_msgs::Twist& cmd) {
        TimedCommandSample sample;
        if (history.sampleAt(stamp, sample)) {
            cmd = sample.cmd;
            return true;
        }
        complete = false;
        cmd = geometry_msgs::Twist();
        return !params.require_complete_history;
    };

    RobotState robot = raw_robot;
    SloshState slosh = raw_slosh;
    ros::Time t = state_epoch;
    double elapsed = 0.0;
    while (elapsed < out.prefix_duration_sec - 1e-9) {
        const double step = std::min(
            params.max_integration_step_sec,
            out.prefix_duration_sec - elapsed);
        geometry_msgs::Twist linear_delayed;
        geometry_msgs::Twist angular_delayed;
        if (!sampleCommand(t - ros::Duration(params.linear_delay_sec),
                           linear_delayed) ||
            !sampleCommand(t - ros::Duration(params.angular_delay_sec),
                           angular_delayed)) {
            out.status = "INCOMPLETE_CMD_HISTORY";
            return out;
        }

        const double linear_target =
            params.linear_gain * linear_delayed.linear.x;
        const double angular_target =
            params.angular_gain * angular_delayed.angular.z;
        const double linear_decay = std::exp(-step / params.linear_tau_sec);
        const double angular_decay = std::exp(-step / params.angular_tau_sec);
        const double next_v =
            linear_target + (robot.v - linear_target) * linear_decay;
        const double next_omega =
            angular_target + (robot.omega - angular_target) * angular_decay;
        const double v_mid = 0.5 * (robot.v + next_v);
        const double omega_mid = 0.5 * (robot.omega + next_omega);
        const double yaw_mid = robot.yaw + 0.5 * omega_mid * step;

        robot.x += v_mid * std::cos(yaw_mid) * step;
        robot.y += v_mid * std::sin(yaw_mid) * step;
        robot.yaw = normalizeYaw(robot.yaw + omega_mid * step);

        if (slosh_configured_) {
            const double a_actual = (next_v - robot.v) / step;
            const double ay_actual = v_mid * omega_mid;
            if (explicit_prefix_slosh_configured_ &&
                std::abs(step -
                         explicit_prefix_slosh_dynamics_.params().dt) <= 1e-12) {
                slosh = explicit_prefix_slosh_dynamics_.step(
                    slosh, a_actual, ay_actual, omega_mid);
            } else {
                SloshState next_slosh;
                if (!slosh_dynamics_.stepWithDt(
                        slosh,
                        a_actual,
                        ay_actual,
                        omega_mid,
                        step,
                        next_slosh)) {
                    out.status = "SLOSH_PREFIX_DISCRETIZATION_FAILED";
                    return out;
                }
                slosh = next_slosh;
            }
        }
        robot.v = next_v;
        robot.omega = next_omega;
        elapsed += step;
        t += ros::Duration(step);
    }

    geometry_msgs::Twist current_cmd;
    if (!sampleCommand(target_epoch, current_cmd)) {
        out.status = "NO_CURRENT_COMMAND";
        return out;
    }
    out.actuator.v_cmd = current_cmd.linear.x;
    out.actuator.omega_cmd = current_cmd.angular.z;

    for (int i = 0; i < kExplicitLinearDelaySteps; ++i) {
        geometry_msgs::Twist cmd;
        const int steps_ago = kExplicitLinearDelaySteps - i;
        if (!sampleCommand(
                target_epoch - ros::Duration(steps_ago * params.dt), cmd)) {
            out.status = "INCOMPLETE_LINEAR_DELAY_QUEUE";
            return out;
        }
        out.actuator.linear_delay_queue[static_cast<size_t>(i)] =
            cmd.linear.x;
    }
    for (int i = 0; i < kExplicitAngularDelaySteps; ++i) {
        geometry_msgs::Twist cmd;
        const int steps_ago = kExplicitAngularDelaySteps - i;
        if (!sampleCommand(
                target_epoch - ros::Duration(steps_ago * params.dt), cmd)) {
            out.status = "INCOMPLETE_ANGULAR_DELAY_QUEUE";
            return out;
        }
        out.actuator.angular_delay_queue[static_cast<size_t>(i)] =
            cmd.angular.z;
    }

    // The queue tail is the final command emitted one OCP interval before
    // target_epoch.  Use final published commands, rather than the previous
    // solver candidate, as the stage-0 acceleration-memory authority.
    out.actuator.a_cmd_memory =
        (out.actuator.v_cmd - out.actuator.linear_delay_queue.back()) /
        params.dt;
    if (!std::isfinite(out.actuator.a_cmd_memory)) {
        out.status = "INVALID_ACCEL_COMMAND_MEMORY";
        return out;
    }
    out.actuator.delayed_v_cmd = out.actuator.linear_delay_queue.front();
    out.actuator.delayed_omega_cmd = out.actuator.angular_delay_queue.front();
    out.actuator.a_actual =
        (params.linear_gain * out.actuator.delayed_v_cmd - robot.v) /
        params.linear_tau_sec;
    out.actuator.alpha_actual =
        (params.angular_gain * out.actuator.delayed_omega_cmd - robot.omega) /
        params.angular_tau_sec;
    out.actuator.valid = true;
    out.predicted_robot = robot;
    out.predicted_slosh = slosh;
    out.history_complete = complete;
    out.valid = complete || !params.require_complete_history;
    out.status = out.valid ? "EXPLICIT_ACTUATOR_READY"
                           : "INCOMPLETE_CMD_HISTORY";
    return out;
}

double ExecutionStatePredictor::normalizeYaw(double yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

}  // namespace spmpc_local_planner
