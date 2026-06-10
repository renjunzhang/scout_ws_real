#pragma once

#include <limits>
#include <string>

#include "spmpc_local_planner/core/terminal_diagnostics.h"

namespace spmpc_local_planner {

struct TerminalControllerParams {
    bool enable = true;
    double goal_tolerance = 0.15;
    bool slowdown_enable = true;
    double slowdown_distance = 1.20;
    double slowdown_v_max = 0.18;
    bool capture_stop_enable = true;
    double capture_stop_distance = 0.70;
    double capture_v_cap = 0.18;
    double goal_behind_x = -0.05;
    double goal_release_distance_margin = 0.10;
    double goal_reached_max_speed = 0.03;
    double goal_reached_max_omega = 0.05;
    bool command_clamp_enable = true;
    bool rate_limit_enable = true;
    bool omega_clamp_enable = true;
    double omega_clamp_max = 0.25;
    double omega_near_goal_max = 0.10;
    double omega_near_goal_distance = 0.35;
};

struct TerminalGoalInfo {
    bool valid = false;
    bool position_reached = false;
    double distance_to_goal = std::numeric_limits<double>::infinity();
    double remaining_s = std::numeric_limits<double>::infinity();
    double dx_robot = std::numeric_limits<double>::quiet_NaN();
};

struct TerminalPlan {
    bool terminal_phase = false;
    bool pre_terminal_phase = true;
    bool envelope_active = false;
    bool stop_pending = false;
    double v_envelope = std::numeric_limits<double>::infinity();
    std::string mode = "NONE";
};

struct TerminalClampOutput {
    double cmd_v_pre = 0.0;
    double cmd_v_post = 0.0;
    double cmd_omega_pre = 0.0;
    double cmd_omega_post = 0.0;
};

class TerminalController {
public:
    void setParams(const TerminalControllerParams& params);
    const TerminalControllerParams& params() const { return params_; }
    void reset();
    void clearPending();

    TerminalPlan updateAndPlan(
        const TerminalGoalInfo& goal,
        double current_v,
        double current_omega,
        double a_brake);

    TerminalClampOutput clampCommand(
        double cmd_v,
        double cmd_omega,
        double current_v,
        double dt,
        const TerminalGoalInfo& goal,
        const TerminalPlan& plan,
        double a_brake);

    bool reached() const { return diagnostics_.reached; }
    const TerminalDiagnostics& diagnostics() const { return diagnostics_; }

private:
    double computeVelocityEnvelope(const TerminalGoalInfo& goal, double a_brake) const;

    TerminalControllerParams params_;
    bool stop_pending_ = false;
    bool reached_latched_ = false;
    TerminalDiagnostics diagnostics_;
};

}  // namespace spmpc_local_planner
