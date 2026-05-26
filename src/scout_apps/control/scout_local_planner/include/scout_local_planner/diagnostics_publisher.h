/**
 * @file diagnostics_publisher.h
 * @brief MPC / slosh / terminal 诊断 topic 发布器
 */

#pragma once

#include "scout_local_planner/types.h"
#include "scout_local_planner/slosh_feedback.h"
#include "scout_local_planner/slosh_integration.h"

#include <ros/ros.h>

#include <Eigen/Dense>

#include <limits>
#include <string>
#include <vector>

namespace scout_local_planner {

struct DiagnosticsCostBreakdown {
    double J_lag = 0.0;
    double J_contour = 0.0;
    double J_etheta = 0.0;
    double J_v = 0.0;
    double J_omega_ff = 0.0;
    double J_control = 0.0;
    double J_smooth = 0.0;
    double J_slosh_eta = 0.0;
    double J_slosh_eta_dot = 0.0;
    double J_total = 0.0;
};

struct TerminalDebugData {
    std::string mode = "NONE";
    GoalInfo goal_info;
    bool goal_info_valid = false;
    double v_envelope = std::numeric_limits<double>::infinity();
    int envelope_active = 0;
    int phase_active = 0;
    double cmd_v_pre_clamp = std::numeric_limits<double>::quiet_NaN();
    double cmd_v_post_clamp = std::numeric_limits<double>::quiet_NaN();
    int profile_cap_active = 0;
    double profile_cap_v_profile = std::numeric_limits<double>::quiet_NaN();
    double profile_cap_cmd_v_pre = std::numeric_limits<double>::quiet_NaN();
    double profile_cap_cmd_v_post = std::numeric_limits<double>::quiet_NaN();
    double profile_cap_implied_ax = std::numeric_limits<double>::quiet_NaN();
    double profile_cap_implied_jerk = std::numeric_limits<double>::quiet_NaN();
};

struct SloshDebugData {
    int episode_id = 0;
    double predicted_height_max = 0.0;
    double q_slosh_eta = 0.0;
    int constraint_active = -1;
    double v_des_eff = 0.0;
    double v_des_raw = 0.0;
    double v_des_target = 0.0;
    int v_des_rate_limited_active = 0;
    SloshFeedbackOutput feedback;
    bool imu_ay_bias_compensation_enable = false;
    bool slosh_enabled = false;
    Eigen::Vector4d slosh_state = Eigen::Vector4d::Zero();
    double omega_n = 0.0;
    double slosh_height = 0.0;
    double solve_time_ms = 0.0;
    bool solve_ok = false;
    bool publish_solver_debug = true;
};

class DiagnosticsPublisher {
public:
    void advertise(ros::NodeHandle& nh);

    void publishCostBreakdown(const DiagnosticsCostBreakdown& breakdown);
    void publishSloshHorizonSummary(const MPCSolution& solution,
                                    bool slosh_enabled,
                                    double h_coeff,
                                    const SloshParams& slosh_params);
    void publishTerminalDebug(const TerminalDebugData& data);
    void publishReferenceExecutionDebug(const std::vector<ReferencePoint>& refs,
                                        double dt);
    void publishSloshDebug(const SloshDebugData& data);

private:
    ros::Publisher slosh_state_pub_;
    ros::Publisher slosh_height_pub_;
    ros::Publisher slosh_ax_est_pub_;
    ros::Publisher slosh_ay_est_pub_;
    ros::Publisher slosh_alpha_est_pub_;
    ros::Publisher slosh_episode_id_pub_;
    ros::Publisher slosh_height_pred_max_pub_;
    ros::Publisher slosh_q_slosh_eta_pub_;
    ros::Publisher slosh_constraint_active_pub_;
    ros::Publisher slosh_v_des_eff_pub_;
    ros::Publisher slosh_omega_est_used_pub_;
    ros::Publisher slosh_imu_omega_z_filtered_pub_;
    ros::Publisher slosh_imu_ay_bias_pub_;
    ros::Publisher slosh_imu_ay_filtered_pub_;
    ros::Publisher slosh_imu_ay_bias_ready_pub_;
    ros::Publisher slosh_eta_norm_pub_;
    ros::Publisher slosh_eta_dot_norm_pub_;
    ros::Publisher slosh_modal_energy_pub_;
    ros::Publisher slosh_modal_energy_norm_pub_;
    ros::Publisher slosh_excitation_ay_abs_pub_;
    ros::Publisher slosh_excitation_alpha_abs_pub_;
    ros::Publisher mpc_solve_ms_pub_;
    ros::Publisher mpc_status_val_pub_;
    ros::Publisher mpc_cost_breakdown_pub_;
    ros::Publisher mpc_slosh_horizon_summary_pub_;
    ros::Publisher terminal_mode_pub_;
    ros::Publisher terminal_recovery_latched_pub_;
    ros::Publisher terminal_goal_info_pub_;
    ros::Publisher terminal_v_envelope_pub_;
    ros::Publisher terminal_envelope_active_pub_;
    ros::Publisher terminal_phase_active_pub_;
    ros::Publisher terminal_cmd_v_pre_clamp_pub_;
    ros::Publisher terminal_cmd_v_post_clamp_pub_;
    ros::Publisher profile_cap_active_pub_;
    ros::Publisher profile_cap_v_profile_pub_;
    ros::Publisher profile_cap_cmd_v_pre_pub_;
    ros::Publisher profile_cap_cmd_v_post_pub_;
    ros::Publisher profile_cap_implied_ax_pub_;
    ros::Publisher profile_cap_implied_jerk_pub_;
    ros::Publisher ref_v_ref_pub_;
    ros::Publisher ref_v_ref_horizon_pub_;
    ros::Publisher ref_s_horizon_pub_;
    ros::Publisher ref_v_des_raw_pub_;
    ros::Publisher ref_v_des_target_pub_;
    ros::Publisher ref_v_des_eff_pub_;
    ros::Publisher ref_v_des_rate_limited_pub_;
    ros::Publisher ref_v_path_pub_;
    ros::Publisher ref_kappa_pub_;
    ros::Publisher ref_s_pub_;
    ros::Publisher ref_implied_ax_pub_;
    ros::Publisher ref_implied_ay_pub_;
    ros::Publisher ref_implied_jerk_pub_;
    ros::Publisher ref_implied_ax_abs_p95_pub_;
    ros::Publisher ref_implied_ay_abs_p95_pub_;
    ros::Publisher ref_implied_jerk_abs_p95_pub_;
};

}  // namespace scout_local_planner
