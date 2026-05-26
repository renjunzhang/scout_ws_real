/**
 * @file terminal_controller.h
 * @brief Terminal approach state and velocity envelope logic.
 */

#pragma once

#include "scout_local_planner/types.h"

#include <limits>
#include <string>

namespace scout_local_planner {

struct TerminalControllerParams {
    double goal_behind_x = -0.05;
    bool slowdown_enable = true;
    double slowdown_distance = 1.20;
    double slowdown_v_max = 0.18;
    double slowdown_q_v = 40.0;
    double slowdown_terminal_factor_v = 5.0;
    bool capture_stop_enable = true;
    double capture_stop_distance = 0.70;
    double capture_v_cap = 0.18;
};

struct TerminalPlan {
    bool envelope_active = false;
    bool terminal_phase = false;
    double v_envelope = std::numeric_limits<double>::infinity();
    double v_des_raw = 0.0;
    double approach_v_cap = 0.0;
};

struct TerminalStateUpdate {
    bool reached = false;
};

struct TerminalClampOutput {
    double cmd_v = 0.0;
    double cmd_v_pre = std::numeric_limits<double>::quiet_NaN();
    double cmd_v_post = std::numeric_limits<double>::quiet_NaN();
};

class TerminalController {
public:
    void setParams(const TerminalControllerParams& params);
    void reset();
    void clearPending();

    bool goalStopPending() const { return goal_stop_pending_; }
    const std::string& modeDebug() const { return mode_debug_; }
    const TerminalControllerParams& params() const { return params_; }

    TerminalStateUpdate updateState(bool has_goal_info,
                                    const GoalInfo& goal_info,
                                    double goal_dist,
                                    double current_v,
                                    double current_omega,
                                    const PathHandlerParams& path_params);

    TerminalPlan plan(bool has_goal_info,
                      const GoalInfo& goal_info,
                      double goal_dist,
                      double v_nominal,
                      const PathHandlerParams& path_params,
                      double a_brake);

    TerminalClampOutput clampCommand(double cmd_v,
                                     double filtered_v,
                                     double dt,
                                     bool has_goal_info,
                                     const GoalInfo& goal_info,
                                     const TerminalPlan& plan,
                                     double a_brake) const;

private:
    static double computeVelocityEnvelope(double goal_dist,
                                          const TerminalControllerParams& params,
                                          double goal_tol,
                                          double a_brake,
                                          bool goal_stop_pending);

    TerminalControllerParams params_;
    bool goal_stop_pending_ = false;
    std::string mode_debug_ = "NONE";
};

}  // namespace scout_local_planner
