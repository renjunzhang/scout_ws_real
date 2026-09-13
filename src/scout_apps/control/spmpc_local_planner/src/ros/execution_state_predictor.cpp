#include "spmpc_local_planner/ros/execution_state_predictor.h"
#include "spmpc_local_planner/dynamics/actual_motion_propagator.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

bool ExecutionStatePredictor::configure(
    const SloshModelParams& slosh_params,
    double explicit_prefix_step_sec) {
    // The generated kernel accepts each interval directly; no discrete-matrix
    // cache or second model configuration is needed for the prefix step.
    slosh_configured_ = std::isfinite(explicit_prefix_step_sec) &&
        explicit_prefix_step_sec >= 0.0 && slosh_dynamics_.configure(slosh_params);
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
        const double alpha_actual = (omega - robot.omega) / std::max(1e-6, step);
        robot.omega = omega;

        if (slosh_configured_) {
            const double ax = (v - prev_v) / std::max(1e-6, step);
            const double ay = v * omega;
            if (!slosh_dynamics_.stepWithDt(slosh, {ax, ay, omega, alpha_actual}, step, slosh)) {
                out.status_code = DelayPhaseStatusCode::DynamicsFailure;
                out.status = delayPhaseStatusName(out.status_code);
                return out;
            }
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
    if (!slosh_configured_ || !finiteSloshState(raw_slosh) ||
        !std::isfinite(raw_robot.x) || !std::isfinite(raw_robot.y) ||
        !std::isfinite(raw_robot.yaw) || !std::isfinite(raw_robot.v) ||
        !std::isfinite(raw_robot.omega)) {
        out.status = "INVALID_ACTUATOR_INITIAL_STATE_OR_MODEL";
        return out;
    }
    if (state_epoch.isZero() || target_epoch.isZero() ||
        target_epoch < state_epoch) {
        out.status = "INVALID_ACTUATOR_EPOCH";
        return out;
    }
    // ROS time is unsigned. During simulation clock startup there may not yet
    // be enough clock history to subtract the calibrated delays.
    if (state_epoch.toSec() < std::max(params.linear_delay_sec, params.angular_delay_sec)) {
        out.status = "INSUFFICIENT_CLOCK_HISTORY";
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
    std::string history_error;
    const auto sampleCommand = [&](const ros::Time& stamp,
                                   geometry_msgs::Twist& cmd, double hold_sec = 0.0) {
        TimedCommandSample sample;
        if (history.sampleAt(stamp, sample)) {
            if (!std::isfinite(sample.cmd.linear.x) || !std::isfinite(sample.cmd.angular.z)) {
                history_error = "NONFINITE_CMD_HISTORY";
                return false;
            }
            // A fresh last publication cannot repair an earlier watchdog-size
            // gap. Reject the unknown execution interval instead of assuming
            // an old nonzero command was held indefinitely.
            if (params.cmd_timeout_sec > 0.0 &&
                (stamp - sample.stamp).toSec() + hold_sec > params.cmd_timeout_sec + 1e-9) {
                history_error = "CMD_HISTORY_GAP";
                return false;
            }
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
    while (t < target_epoch) {
        ros::Time end = std::min(target_epoch, t + ros::Duration(params.max_integration_step_sec));
        // Both channels are ZOH at their own delayed publication edges.
        // Split there before integrating; a smaller RK4 step alone cannot
        // correct holding the wrong command across a discontinuity.
        for (double delay : {params.linear_delay_sec, params.angular_delay_sec}) {
            ros::Time next_stamp;
            if (history.nextStampAfter(t - ros::Duration(delay), next_stamp))
                end = std::min(end, next_stamp + ros::Duration(delay));
        }
        const double step = (end - t).toSec();
        if (step <= 0.0) { out.status = "INVALID_PREFIX_STEP"; return out; }
        geometry_msgs::Twist linear_delayed;
        geometry_msgs::Twist angular_delayed;
        if (!sampleCommand(t - ros::Duration(params.linear_delay_sec),
                           linear_delayed, step) ||
            !sampleCommand(t - ros::Duration(params.angular_delay_sec),
                           angular_delayed, step)) {
            out.status = history_error.empty() ? "INCOMPLETE_CMD_HISTORY" : history_error;
            return out;
        }

        if (!propagateActualMotion(
                robot, slosh, {linear_delayed.linear.x, angular_delayed.angular.z},
                params, slosh_dynamics_, step)) {
            out.status = "COUPLED_ACTUATOR_LIQUID_PROPAGATION_FAILED";
            return out;
        }
        robot.yaw = normalizeYaw(robot.yaw);
        t = end;
    }

    geometry_msgs::Twist current_cmd;
    if (!sampleCommand(target_epoch, current_cmd)) {
        out.status = history_error.empty() ? "NO_CURRENT_COMMAND" : history_error;
        return out;
    }
    out.actuator.v_cmd = current_cmd.linear.x;
    out.actuator.omega_cmd = current_cmd.angular.z;

    for (int i = 0; i < kExplicitLinearDelaySteps; ++i) {
        geometry_msgs::Twist cmd;
        const int steps_ago = kExplicitLinearDelaySteps - i;
        if (!sampleCommand(
                target_epoch - ros::Duration(steps_ago * params.dt), cmd)) {
            out.status = history_error.empty() ? "INCOMPLETE_LINEAR_DELAY_QUEUE" : history_error;
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
            out.status = history_error.empty() ? "INCOMPLETE_ANGULAR_DELAY_QUEUE" : history_error;
            return out;
        }
        out.actuator.angular_delay_queue[static_cast<size_t>(i)] =
            cmd.angular.z;
    }

    // Acceleration memory belongs to consecutive emitted commands, whereas
    // the delay FIFO is sampled by time. Under publication jitter the FIFO tail
    // may contain the current command or skip a publication; it is not the
    // previous command in the OCP's discrete acceleration recurrence.
    TimedCommandSample current_sample, previous_sample;
    history.sampleAt(target_epoch, current_sample);
    if (history.sampleBefore(current_sample.stamp, previous_sample)) {
        if (!std::isfinite(previous_sample.cmd.linear.x)) {
            out.status = "NONFINITE_CMD_HISTORY";
            return out;
        }
        out.actuator.a_cmd_memory = (current_cmd.linear.x - previous_sample.cmd.linear.x) / params.dt;
    } else {
        // A lone command can establish a steady history only after one whole
        // nominal interval. Never invent its predecessor at startup.
        if ((target_epoch - current_sample.stamp).toSec() < params.dt) {
            out.status = "INCOMPLETE_ACCEL_COMMAND_HISTORY";
            return out;
        }
        out.actuator.a_cmd_memory = 0.0;
    }
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
