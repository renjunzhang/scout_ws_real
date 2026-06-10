#include "spmpc_local_planner/core/terminal_controller.h"
#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

bool finite(double value) {
    return std::isfinite(value);
}

}  // namespace

void TerminalController::setParams(const TerminalControllerParams& params) {
    params_ = params;
    reset();
}

void TerminalController::reset() {
    stop_pending_ = false;
    reached_latched_ = false;
    diagnostics_ = TerminalDiagnostics{};
    diagnostics_.enabled = params_.enable;
}

void TerminalController::clearPending() {
    stop_pending_ = false;
}

TerminalPlan TerminalController::updateAndPlan(
    const TerminalGoalInfo& goal,
    double current_v,
    double current_omega,
    double a_brake) {
    diagnostics_ = TerminalDiagnostics{};
    diagnostics_.enabled = params_.enable;
    diagnostics_.distance_to_goal = goal.distance_to_goal;
    diagnostics_.remaining_s = goal.remaining_s;
    diagnostics_.dx_robot = goal.dx_robot;
    diagnostics_.position_reached = goal.position_reached;
    diagnostics_.speed_gate_reached = std::abs(current_v) <= params_.goal_reached_max_speed;
    diagnostics_.omega_gate_reached = std::abs(current_omega) <= params_.goal_reached_max_omega;

    TerminalPlan plan;
    if (!params_.enable || !goal.valid) {
        plan.mode = params_.enable ? "NO_GOAL" : "DISABLED";
        diagnostics_.mode = plan.mode;
        return plan;
    }

    if (reached_latched_) {
        stop_pending_ = true;
        plan.stop_pending = true;
        plan.envelope_active = true;
        plan.terminal_phase = true;
        plan.pre_terminal_phase = false;
        plan.v_envelope = 0.0;
        plan.mode = "REACHED";

        diagnostics_.terminal_phase = plan.terminal_phase;
        diagnostics_.pre_terminal_phase = plan.pre_terminal_phase;
        diagnostics_.envelope_active = plan.envelope_active;
        diagnostics_.stop_pending = plan.stop_pending;
        diagnostics_.v_envelope = plan.v_envelope;
        diagnostics_.reached = true;
        diagnostics_.mode = plan.mode;
        return plan;
    }

    const double release_distance = params_.capture_stop_distance + params_.goal_release_distance_margin;
    if (stop_pending_ && goal.distance_to_goal > release_distance && !(finite(goal.dx_robot) && goal.dx_robot < params_.goal_behind_x)) {
        stop_pending_ = false;
    }

    const bool capture_reached = params_.capture_stop_enable && goal.distance_to_goal <= params_.capture_stop_distance;
    if (goal.position_reached || capture_reached) {
        stop_pending_ = true;
    }

    const double envelope = computeVelocityEnvelope(goal, a_brake);
    const bool envelope_active = finite(envelope);
    const bool terminal_phase = stop_pending_ || envelope_active;

    plan.stop_pending = stop_pending_;
    plan.envelope_active = envelope_active;
    plan.terminal_phase = terminal_phase;
    plan.pre_terminal_phase = !terminal_phase;
    plan.v_envelope = envelope;
    if (goal.position_reached && diagnostics_.speed_gate_reached && diagnostics_.omega_gate_reached) {
        reached_latched_ = true;
        plan.mode = "REACHED";
        diagnostics_.reached = true;
    } else if (stop_pending_) {
        plan.mode = "TERMINAL_CAPTURE_STOP";
    } else if (envelope_active) {
        plan.mode = "TERMINAL_SLOWDOWN";
    } else {
        plan.mode = "TRACKING";
    }

    diagnostics_.terminal_phase = plan.terminal_phase;
    diagnostics_.pre_terminal_phase = plan.pre_terminal_phase;
    diagnostics_.envelope_active = plan.envelope_active;
    diagnostics_.stop_pending = plan.stop_pending;
    diagnostics_.v_envelope = plan.v_envelope;
    diagnostics_.mode = plan.mode;
    return plan;
}

TerminalClampOutput TerminalController::clampCommand(
    double cmd_v,
    double cmd_omega,
    double current_v,
    double dt,
    const TerminalGoalInfo& goal,
    const TerminalPlan& plan,
    double a_brake) {
    TerminalClampOutput out;
    out.cmd_v_pre = cmd_v;
    out.cmd_omega_pre = cmd_omega;
    out.cmd_v_post = cmd_v;
    out.cmd_omega_post = cmd_omega;

    if (!params_.enable || !params_.command_clamp_enable || !goal.valid || !plan.terminal_phase) {
        diagnostics_.cmd_v_pre_clamp = out.cmd_v_pre;
        diagnostics_.cmd_v_post_clamp = out.cmd_v_post;
        return out;
    }

    const bool goal_behind = finite(goal.dx_robot) && goal.dx_robot < params_.goal_behind_x;
    double target_v = cmd_v;
    if (goal_behind && plan.stop_pending) {
        out.cmd_v_post = 0.0;
        out.cmd_omega_post = 0.0;
        diagnostics_.cmd_v_pre_clamp = out.cmd_v_pre;
        diagnostics_.cmd_v_post_clamp = out.cmd_v_post;
        return out;
    } else if (finite(plan.v_envelope)) {
        target_v = std::min(target_v, plan.v_envelope);
    }

    target_v = std::max(0.0, target_v);
    if (params_.rate_limit_enable && dt > 1e-6) {
        const double brake_step = std::max(0.0, a_brake) * dt;
        target_v = std::max(target_v, std::max(0.0, current_v - brake_step));
    }
    out.cmd_v_post = target_v;

    if (params_.omega_clamp_enable) {
        double omega_limit = std::max(0.0, params_.omega_clamp_max);
        if (goal.distance_to_goal <= params_.omega_near_goal_distance) {
            omega_limit = std::min(omega_limit, std::max(0.0, params_.omega_near_goal_max));
        }
        out.cmd_omega_post = clampValue(out.cmd_omega_post, -omega_limit, omega_limit);
    }

    diagnostics_.cmd_v_pre_clamp = out.cmd_v_pre;
    diagnostics_.cmd_v_post_clamp = out.cmd_v_post;
    return out;
}

double TerminalController::computeVelocityEnvelope(const TerminalGoalInfo& goal, double a_brake) const {
    if (!params_.enable || !goal.valid) {
        return std::numeric_limits<double>::infinity();
    }
    if (stop_pending_) {
        const double brake_dist = std::max(0.0, goal.distance_to_goal - params_.goal_tolerance);
        const double brake_cap = std::sqrt(std::max(0.0, 2.0 * std::max(1e-6, a_brake) * brake_dist));
        return std::min(std::max(0.0, params_.capture_v_cap), brake_cap);
    }
    if (params_.capture_stop_enable && goal.distance_to_goal <= params_.capture_stop_distance) {
        return std::max(0.0, params_.capture_v_cap);
    }
    if (!params_.slowdown_enable || goal.distance_to_goal > params_.slowdown_distance) {
        return std::numeric_limits<double>::infinity();
    }

    const double denom = std::max(1e-6, params_.slowdown_distance - params_.goal_tolerance);
    const double ratio = clampValue((goal.distance_to_goal - params_.goal_tolerance) / denom, 0.0, 1.0);
    const double cap = std::max(0.0, params_.slowdown_v_max) * std::max(0.2, ratio);
    return cap;
}

}  // namespace spmpc_local_planner
