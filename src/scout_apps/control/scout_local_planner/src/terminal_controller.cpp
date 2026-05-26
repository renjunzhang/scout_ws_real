#include "scout_local_planner/terminal_controller.h"

#include <algorithm>
#include <cmath>

namespace scout_local_planner {

namespace {

double limitRate(double target, double current, double rate_limit, double dt) {
    if (!std::isfinite(target) || !std::isfinite(current) ||
        rate_limit <= 1e-6 || dt <= 1e-6) {
        return target;
    }
    const double max_delta = rate_limit * dt;
    return std::max(current - max_delta, std::min(current + max_delta, target));
}

}  // namespace

void TerminalController::setParams(const TerminalControllerParams& params) {
    params_ = params;
}

void TerminalController::reset() {
    goal_stop_pending_ = false;
    mode_debug_ = "NONE";
}

void TerminalController::clearPending() {
    goal_stop_pending_ = false;
}

TerminalStateUpdate TerminalController::updateState(
    bool has_goal_info,
    const GoalInfo& goal_info,
    double goal_dist,
    double current_v,
    double current_omega,
    const PathHandlerParams& path_params) {
    TerminalStateUpdate out;
    const bool goal_position_reached =
        has_goal_info && goal_info.valid && goal_info.position_reached;
    const double goal_stop_release_dist =
        std::max({path_params.goal_capture_distance,
                  path_params.goal_tolerance + 0.15,
                  params_.capture_stop_enable ? params_.capture_stop_distance + 0.10 : 0.0});
    const bool terminal_capture_stop_reached =
        params_.capture_stop_enable &&
        std::isfinite(goal_dist) &&
        goal_dist < params_.capture_stop_distance;

    if (goal_stop_pending_) {
        const bool goal_behind =
            has_goal_info &&
            goal_info.valid &&
            std::isfinite(goal_info.dx) &&
            goal_info.dx < params_.goal_behind_x;
        const bool should_release =
            !std::isfinite(goal_dist) ||
            (goal_dist > goal_stop_release_dist && !goal_behind);
        if (should_release) {
            goal_stop_pending_ = false;
        }
    }

    if (!goal_stop_pending_ && !goal_position_reached && !terminal_capture_stop_reached) {
        mode_debug_ = "NONE";
        return out;
    }

    if (goal_position_reached || terminal_capture_stop_reached) {
        goal_stop_pending_ = true;
        mode_debug_ = "TERMINAL_MPC_STOP";
    }

    const bool speed_low =
        std::abs(current_v) < path_params.goal_reached_max_speed &&
        std::abs(current_omega) < path_params.goal_reached_max_omega;
    if (speed_low && goal_position_reached) {
        out.reached = true;
        goal_stop_pending_ = false;
        mode_debug_ = "REACHED";
    }
    return out;
}

TerminalPlan TerminalController::plan(
    bool has_goal_info,
    const GoalInfo& goal_info,
    double goal_dist,
    double v_nominal,
    const PathHandlerParams& path_params,
    double a_brake) {
    TerminalPlan out;
    out.approach_v_cap =
        params_.capture_v_cap > 1e-6
            ? params_.capture_v_cap
            : std::max(0.0, params_.slowdown_v_max);
    out.v_envelope = params_.slowdown_enable
        ? computeVelocityEnvelope(
              goal_dist, params_, path_params.goal_tolerance, a_brake, goal_stop_pending_)
        : std::numeric_limits<double>::infinity();
    out.envelope_active = std::isfinite(out.v_envelope);
    out.terminal_phase = goal_stop_pending_ || out.envelope_active;

    const bool goal_position_reached =
        has_goal_info && goal_info.valid && goal_info.position_reached;
    const bool goal_behind =
        has_goal_info &&
        goal_info.valid &&
        std::isfinite(goal_info.dx) &&
        goal_info.dx <= 0.0;
    out.v_des_raw =
        (goal_stop_pending_ && (goal_position_reached || goal_behind)) ? 0.0 :
        goal_stop_pending_ ? out.approach_v_cap :
        v_nominal;

    mode_debug_ = goal_stop_pending_ ? "TERMINAL_MPC_STOP" : "NONE";
    return out;
}

TerminalClampOutput TerminalController::clampCommand(
    double cmd_v,
    double filtered_v,
    double dt,
    bool has_goal_info,
    const GoalInfo& goal_info,
    const TerminalPlan& plan,
    double a_brake) const {
    TerminalClampOutput out;
    out.cmd_v = cmd_v;
    out.cmd_v_pre = cmd_v;

    if (!plan.terminal_phase) {
        out.cmd_v_post = cmd_v;
        return out;
    }

    bool goal_behind = false;
    out.cmd_v = std::min<double>(cmd_v, plan.v_envelope);
    if (has_goal_info &&
        goal_info.valid &&
        std::isfinite(goal_info.dx) &&
        goal_info.dx <= 0.0) {
        goal_behind = true;
        out.cmd_v = 0.0;
    }
    out.cmd_v = std::max(0.0, out.cmd_v);
    if (!goal_behind) {
        out.cmd_v = limitRate(
            out.cmd_v,
            std::max(0.0, filtered_v),
            std::max(1e-6, a_brake),
            dt);
    }
    out.cmd_v_post = out.cmd_v;
    return out;
}

double TerminalController::computeVelocityEnvelope(
    double goal_dist,
    const TerminalControllerParams& params,
    double goal_tol,
    double a_brake,
    bool goal_stop_pending) {
    if (!std::isfinite(goal_dist)) {
        return std::numeric_limits<double>::infinity();
    }
    const double a = std::max(1e-6, a_brake);
    const double v_cap = std::max(0.0, params.slowdown_v_max);
    const double approach_cap = params.capture_v_cap > 1e-6
        ? params.capture_v_cap
        : v_cap;

    if (goal_stop_pending) {
        const double remain = std::max(0.0, goal_dist - goal_tol);
        const double v_kinematic = std::sqrt(2.0 * a * remain);
        return std::min(approach_cap, v_kinematic);
    }

    if (goal_dist >= params.slowdown_distance) {
        return std::numeric_limits<double>::infinity();
    }

    if (params.capture_stop_enable &&
        params.capture_stop_distance > goal_tol + 1e-3) {
        const double remain_to_capture =
            std::max(0.0, goal_dist - params.capture_stop_distance);
        const double v_kinematic =
            std::sqrt(approach_cap * approach_cap + 2.0 * a * remain_to_capture);
        return std::min(v_cap, v_kinematic);
    }

    const double remain = std::max(0.0, goal_dist - goal_tol);
    const double v_kinematic = std::sqrt(2.0 * a * remain);
    return std::min(v_cap, v_kinematic);
}

}  // namespace scout_local_planner
