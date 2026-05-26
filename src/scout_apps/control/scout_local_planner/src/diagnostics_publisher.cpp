/**
 * @file diagnostics_publisher.cpp
 * @brief MPC / slosh / terminal 诊断 topic 发布器实现
 */

#include "scout_local_planner/diagnostics_publisher.h"

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>

namespace scout_local_planner {

namespace {

double percentile(std::vector<double> samples, double p) {
    if (samples.empty()) {
        return 0.0;
    }
    std::sort(samples.begin(), samples.end());
    const double alpha = std::max(0.0, std::min(1.0, p));
    const std::size_t idx = static_cast<std::size_t>(
        std::round(alpha * static_cast<double>(samples.size() - 1)));
    return samples[std::min(idx, samples.size() - 1)];
}

void publishFloat(ros::Publisher& pub, double value) {
    if (pub.getNumSubscribers() <= 0) {
        return;
    }
    std_msgs::Float32 msg;
    msg.data = static_cast<float>(value);
    pub.publish(msg);
}

void publishInt(ros::Publisher& pub, int value) {
    if (pub.getNumSubscribers() <= 0) {
        return;
    }
    std_msgs::Int32 msg;
    msg.data = value;
    pub.publish(msg);
}

double absP95(std::vector<double> values) {
    values.erase(
        std::remove_if(values.begin(), values.end(),
                       [](double v) { return !std::isfinite(v); }),
        values.end());
    if (values.empty()) {
        return 0.0;
    }
    for (double& value : values) {
        value = std::abs(value);
    }
    std::sort(values.begin(), values.end());
    const double pos = 0.95 * static_cast<double>(values.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = static_cast<size_t>(std::ceil(pos));
    if (lo == hi) {
        return values[lo];
    }
    const double r = pos - static_cast<double>(lo);
    return values[lo] * (1.0 - r) + values[hi] * r;
}

}  // namespace

void DiagnosticsPublisher::advertise(ros::NodeHandle& nh) {
    slosh_state_pub_ = nh.advertise<std_msgs::Float32MultiArray>("slosh/state", 1);
    slosh_height_pub_ = nh.advertise<std_msgs::Float32>("slosh/height", 1);
    slosh_ax_est_pub_ = nh.advertise<std_msgs::Float32>("slosh/ax_est", 1);
    slosh_ay_est_pub_ = nh.advertise<std_msgs::Float32>("slosh/ay_est", 1);
    slosh_alpha_est_pub_ = nh.advertise<std_msgs::Float32>("slosh/alpha_est", 1);
    slosh_episode_id_pub_ = nh.advertise<std_msgs::Int32>("slosh/episode_id", 1);
    slosh_height_pred_max_pub_ = nh.advertise<std_msgs::Float32>("slosh/height_pred_max", 1);
    slosh_q_slosh_eta_pub_ = nh.advertise<std_msgs::Float32>("slosh/q_slosh_eta", 1);
    slosh_constraint_active_pub_ = nh.advertise<std_msgs::Int32>("slosh/constraint_active", 1);
    slosh_v_des_eff_pub_ = nh.advertise<std_msgs::Float32>("slosh/v_des_eff", 1);
    slosh_omega_est_used_pub_ = nh.advertise<std_msgs::Float32>("slosh/omega_est_used", 1);
    slosh_imu_omega_z_filtered_pub_ = nh.advertise<std_msgs::Float32>("slosh/imu_omega_z_filtered", 1);
    slosh_imu_ay_bias_pub_ = nh.advertise<std_msgs::Float32>("slosh/imu_ay_bias", 1);
    slosh_imu_ay_filtered_pub_ = nh.advertise<std_msgs::Float32>("slosh/imu_ay_filtered", 1);
    slosh_imu_ay_bias_ready_pub_ = nh.advertise<std_msgs::Int32>("slosh/imu_ay_bias_ready", 1);
    slosh_eta_norm_pub_ = nh.advertise<std_msgs::Float32>("slosh/eta_norm", 1);
    slosh_eta_dot_norm_pub_ = nh.advertise<std_msgs::Float32>("slosh/eta_dot_norm", 1);
    slosh_modal_energy_pub_ = nh.advertise<std_msgs::Float32>("slosh/modal_energy", 1);
    slosh_modal_energy_norm_pub_ = nh.advertise<std_msgs::Float32>("slosh/modal_energy_norm", 1);
    slosh_excitation_ay_abs_pub_ = nh.advertise<std_msgs::Float32>("slosh/excitation_ay_abs", 1);
    slosh_excitation_alpha_abs_pub_ = nh.advertise<std_msgs::Float32>("slosh/excitation_alpha_abs", 1);
    mpc_solve_ms_pub_ = nh.advertise<std_msgs::Float32>("mpc/solve_ms", 1);
    mpc_status_val_pub_ = nh.advertise<std_msgs::Int32>("mpc/status_val", 1);
    mpc_cost_breakdown_pub_ = nh.advertise<std_msgs::Float32MultiArray>("mpc/cost_breakdown", 1);
    mpc_slosh_horizon_summary_pub_ =
        nh.advertise<std_msgs::Float32MultiArray>("mpc/slosh_horizon_summary", 1);
    terminal_mode_pub_ = nh.advertise<std_msgs::String>("terminal/mode", 1);
    terminal_recovery_latched_pub_ = nh.advertise<std_msgs::Int32>("terminal/recovery_latched", 1);
    terminal_goal_info_pub_ = nh.advertise<std_msgs::Float32MultiArray>("terminal/goal_info", 1);
    terminal_v_envelope_pub_ = nh.advertise<std_msgs::Float32>("terminal/v_envelope", 1);
    terminal_envelope_active_pub_ = nh.advertise<std_msgs::Int32>("terminal/envelope_active", 1);
    terminal_phase_active_pub_ = nh.advertise<std_msgs::Int32>("terminal/phase_active", 1);
    terminal_cmd_v_pre_clamp_pub_ =
        nh.advertise<std_msgs::Float32>("terminal/cmd_v_pre_clamp", 1);
    terminal_cmd_v_post_clamp_pub_ =
        nh.advertise<std_msgs::Float32>("terminal/cmd_v_post_clamp", 1);
    profile_cap_active_pub_ = nh.advertise<std_msgs::Int32>("profile_cap/active", 1);
    profile_cap_v_profile_pub_ = nh.advertise<std_msgs::Float32>("profile_cap/v_profile", 1);
    profile_cap_cmd_v_pre_pub_ = nh.advertise<std_msgs::Float32>("profile_cap/cmd_v_pre_cap", 1);
    profile_cap_cmd_v_post_pub_ = nh.advertise<std_msgs::Float32>("profile_cap/cmd_v_post_cap", 1);
    profile_cap_implied_ax_pub_ = nh.advertise<std_msgs::Float32>("profile_cap/implied_ax", 1);
    profile_cap_implied_jerk_pub_ = nh.advertise<std_msgs::Float32>("profile_cap/implied_jerk", 1);
    ref_v_ref_pub_ = nh.advertise<std_msgs::Float32>("reference/v_ref", 1);
    ref_v_ref_horizon_pub_ = nh.advertise<std_msgs::Float32MultiArray>("reference/v_ref_horizon", 1);
    ref_s_horizon_pub_ = nh.advertise<std_msgs::Float32MultiArray>("reference/s_horizon", 1);
    ref_v_des_raw_pub_ = nh.advertise<std_msgs::Float32>("reference/v_des_raw", 1);
    ref_v_des_target_pub_ = nh.advertise<std_msgs::Float32>("reference/v_des_target", 1);
    ref_v_des_eff_pub_ = nh.advertise<std_msgs::Float32>("reference/v_des_eff", 1);
    ref_v_des_rate_limited_pub_ = nh.advertise<std_msgs::Int32>("reference/v_des_rate_limited", 1);
    ref_v_path_pub_ = nh.advertise<std_msgs::Float32>("reference/v_path", 1);
    ref_kappa_pub_ = nh.advertise<std_msgs::Float32>("reference/kappa", 1);
    ref_s_pub_ = nh.advertise<std_msgs::Float32>("reference/s", 1);
    ref_implied_ax_pub_ = nh.advertise<std_msgs::Float32>("reference/implied_ax", 1);
    ref_implied_ay_pub_ = nh.advertise<std_msgs::Float32>("reference/implied_ay", 1);
    ref_implied_jerk_pub_ = nh.advertise<std_msgs::Float32>("reference/implied_jerk", 1);
    ref_implied_ax_abs_p95_pub_ = nh.advertise<std_msgs::Float32>("reference/implied_ax_abs_p95", 1);
    ref_implied_ay_abs_p95_pub_ = nh.advertise<std_msgs::Float32>("reference/implied_ay_abs_p95", 1);
    ref_implied_jerk_abs_p95_pub_ = nh.advertise<std_msgs::Float32>("reference/implied_jerk_abs_p95", 1);
}

void DiagnosticsPublisher::publishCostBreakdown(const DiagnosticsCostBreakdown& breakdown) {
    if (mpc_cost_breakdown_pub_.getNumSubscribers() == 0) {
        return;
    }

    const double total = breakdown.J_total;
    auto pct = [total](double value) {
        return total > 1e-12 ? 100.0 * value / total : 0.0;
    };

    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "total,J_lag,J_contour,J_etheta,J_v,J_omega_ff,J_control,J_smooth,"
        "J_slosh_eta,J_slosh_eta_dot,"
        "pct_lag,pct_contour,pct_etheta,pct_v,pct_omega_ff,pct_control,"
        "pct_smooth,pct_slosh_eta,pct_slosh_eta_dot,pct_slosh_total";
    msg.layout.dim[0].size = 20;
    msg.layout.dim[0].stride = 20;
    msg.data.resize(20);
    msg.data[0] = static_cast<float>(breakdown.J_total);
    msg.data[1] = static_cast<float>(breakdown.J_lag);
    msg.data[2] = static_cast<float>(breakdown.J_contour);
    msg.data[3] = static_cast<float>(breakdown.J_etheta);
    msg.data[4] = static_cast<float>(breakdown.J_v);
    msg.data[5] = static_cast<float>(breakdown.J_omega_ff);
    msg.data[6] = static_cast<float>(breakdown.J_control);
    msg.data[7] = static_cast<float>(breakdown.J_smooth);
    msg.data[8] = static_cast<float>(breakdown.J_slosh_eta);
    msg.data[9] = static_cast<float>(breakdown.J_slosh_eta_dot);
    msg.data[10] = static_cast<float>(pct(breakdown.J_lag));
    msg.data[11] = static_cast<float>(pct(breakdown.J_contour));
    msg.data[12] = static_cast<float>(pct(breakdown.J_etheta));
    msg.data[13] = static_cast<float>(pct(breakdown.J_v));
    msg.data[14] = static_cast<float>(pct(breakdown.J_omega_ff));
    msg.data[15] = static_cast<float>(pct(breakdown.J_control));
    msg.data[16] = static_cast<float>(pct(breakdown.J_smooth));
    msg.data[17] = static_cast<float>(pct(breakdown.J_slosh_eta));
    msg.data[18] = static_cast<float>(pct(breakdown.J_slosh_eta_dot));
    msg.data[19] = static_cast<float>(pct(breakdown.J_slosh_eta + breakdown.J_slosh_eta_dot));
    mpc_cost_breakdown_pub_.publish(msg);
}

void DiagnosticsPublisher::publishSloshHorizonSummary(
    const MPCSolution& solution,
    bool slosh_enabled,
    double h_coeff,
    const SloshParams& slosh_params) {
    if (mpc_slosh_horizon_summary_pub_.getNumSubscribers() == 0) {
        return;
    }

    std_msgs::Float32MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label =
        "eta_norm_0_m,eta_norm_max_m,eta_dot_norm_0_mps,eta_dot_norm_max_mps,"
        "h_modal_max_mm,h_total_max_mm,k_h_total_max,"
        "v_abs_p95_mps,omega_abs_p95_radps,ax_abs_p95_mps2,ay_abs_p95_mps2,"
        "eta_growth_ratio,h_total_0_mm";
    msg.layout.dim[0].size = 13;
    msg.layout.dim[0].stride = 13;
    msg.data.assign(13, 0.0f);

    if (!slosh_enabled || solution.x_predicted.empty()) {
        mpc_slosh_horizon_summary_pub_.publish(msg);
        return;
    }

    const double R = slosh_params.container_radius;
    const double g = 9.81;

    double eta_norm_0 = 0.0;
    double eta_norm_max = 0.0;
    double eta_dot_norm_0 = 0.0;
    double eta_dot_norm_max = 0.0;
    double h_modal_max = 0.0;
    double h_total_max = 0.0;
    double h_total_0 = 0.0;
    std::size_t k_h_total_max = 0;
    std::vector<double> v_abs;
    std::vector<double> omega_abs;
    std::vector<double> ax_abs;
    std::vector<double> ay_abs;

    const std::size_t n_states = solution.x_predicted.size();
    const std::size_t n_inputs = solution.u_optimal.size();
    v_abs.reserve(n_states);
    omega_abs.reserve(n_inputs);
    ax_abs.reserve(n_inputs);
    ay_abs.reserve(n_inputs);

    for (std::size_t k = 0; k < n_states; ++k) {
        const StateVector& xk = solution.x_predicted[k];
        const double eta_x = xk(StateIndex::ETA_X);
        const double eta_y = xk(StateIndex::ETA_Y);
        const double eta_x_dot = xk(StateIndex::ETA_X_DOT);
        const double eta_y_dot = xk(StateIndex::ETA_Y_DOT);
        const double eta_norm = std::hypot(eta_x, eta_y);
        const double eta_dot_norm = std::hypot(eta_x_dot, eta_y_dot);
        const double h_modal = h_coeff * eta_norm;

        double omega_k = 0.0;
        if (n_inputs > 0) {
            omega_k = (k < n_inputs)
                          ? solution.u_optimal[k](ControlIndex::OMEGA)
                          : solution.u_optimal.back()(ControlIndex::OMEGA);
        }

        double h_parabola = 0.0;
        if (slosh_params.use_parabola_term) {
            h_parabola = (R * R * omega_k * omega_k) / (4.0 * g);
        }
        const double h_total = h_modal + h_parabola;

        if (k == 0) {
            eta_norm_0 = eta_norm;
            eta_dot_norm_0 = eta_dot_norm;
            h_total_0 = h_total;
        }
        eta_norm_max = std::max(eta_norm_max, eta_norm);
        eta_dot_norm_max = std::max(eta_dot_norm_max, eta_dot_norm);
        h_modal_max = std::max(h_modal_max, h_modal);
        if (h_total > h_total_max) {
            h_total_max = h_total;
            k_h_total_max = k;
        }
        v_abs.push_back(std::abs(xk(StateIndex::V)));
    }

    for (std::size_t k = 0; k < n_inputs; ++k) {
        const ControlVector& uk = solution.u_optimal[k];
        const double a = uk(ControlIndex::A);
        const double omega = uk(ControlIndex::OMEGA);
        const double v = solution.x_predicted[std::min(k, n_states - 1)](StateIndex::V);
        omega_abs.push_back(std::abs(omega));
        ax_abs.push_back(std::abs(a));
        ay_abs.push_back(std::abs(v * omega));
    }

    const double eta_growth_ratio =
        eta_norm_max / std::max(eta_norm_0, 1e-6);

    msg.data[0] = static_cast<float>(eta_norm_0);
    msg.data[1] = static_cast<float>(eta_norm_max);
    msg.data[2] = static_cast<float>(eta_dot_norm_0);
    msg.data[3] = static_cast<float>(eta_dot_norm_max);
    msg.data[4] = static_cast<float>(1000.0 * h_modal_max);
    msg.data[5] = static_cast<float>(1000.0 * h_total_max);
    msg.data[6] = static_cast<float>(k_h_total_max);
    msg.data[7] = static_cast<float>(percentile(v_abs, 0.95));
    msg.data[8] = static_cast<float>(percentile(omega_abs, 0.95));
    msg.data[9] = static_cast<float>(percentile(ax_abs, 0.95));
    msg.data[10] = static_cast<float>(percentile(ay_abs, 0.95));
    msg.data[11] = static_cast<float>(eta_growth_ratio);
    msg.data[12] = static_cast<float>(1000.0 * h_total_0);
    mpc_slosh_horizon_summary_pub_.publish(msg);
}

void DiagnosticsPublisher::publishTerminalDebug(const TerminalDebugData& data) {
    if (terminal_mode_pub_.getNumSubscribers() > 0) {
        std_msgs::String msg;
        msg.data = data.mode;
        terminal_mode_pub_.publish(msg);
    }

    publishInt(terminal_recovery_latched_pub_, 0);
    publishFloat(terminal_v_envelope_pub_, data.v_envelope);
    publishInt(terminal_envelope_active_pub_, data.envelope_active);
    publishInt(terminal_phase_active_pub_, data.phase_active);
    publishFloat(terminal_cmd_v_pre_clamp_pub_, data.cmd_v_pre_clamp);
    publishFloat(terminal_cmd_v_post_clamp_pub_, data.cmd_v_post_clamp);
    publishInt(profile_cap_active_pub_, data.profile_cap_active);
    publishFloat(profile_cap_v_profile_pub_, data.profile_cap_v_profile);
    publishFloat(profile_cap_cmd_v_pre_pub_, data.profile_cap_cmd_v_pre);
    publishFloat(profile_cap_cmd_v_post_pub_, data.profile_cap_cmd_v_post);
    publishFloat(profile_cap_implied_ax_pub_, data.profile_cap_implied_ax);
    publishFloat(profile_cap_implied_jerk_pub_, data.profile_cap_implied_jerk);

    if (terminal_goal_info_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32MultiArray msg;
        msg.data.resize(8, 0.0f);

        if (!data.goal_info_valid) {
            const float nan = std::numeric_limits<float>::quiet_NaN();
            msg.data[0] = nan;
            msg.data[1] = nan;
            msg.data[2] = nan;
            msg.data[3] = nan;
            msg.data[4] = nan;
        } else {
            msg.data[0] = static_cast<float>(data.goal_info.dx);
            msg.data[1] = static_cast<float>(data.goal_info.dy);
            msg.data[2] = static_cast<float>(data.goal_info.dist);
            msg.data[3] = static_cast<float>(data.goal_info.bearing);
            msg.data[4] = static_cast<float>(data.goal_info.goal_yaw_err);
            msg.data[5] = data.goal_info.has_goal_yaw ? 1.0f : 0.0f;
            msg.data[6] = data.goal_info.position_reached ? 1.0f : 0.0f;
            msg.data[7] = data.goal_info.pose_reached ? 1.0f : 0.0f;
        }

        terminal_goal_info_pub_.publish(msg);
    }
}

void DiagnosticsPublisher::publishReferenceExecutionDebug(
    const std::vector<ReferencePoint>& refs,
    double dt) {
    if (refs.empty()) {
        return;
    }

    const double dt_safe = std::max(1e-6, dt);
    std::vector<double> ax_values;
    std::vector<double> ay_values;
    std::vector<double> jerk_values;
    ax_values.reserve(refs.size());
    ay_values.reserve(refs.size());
    jerk_values.reserve(refs.size());

    double first_ax = 0.0;
    double first_jerk = 0.0;
    bool has_first_ax = false;
    double prev_ax = 0.0;
    bool has_prev_ax = false;

    for (size_t i = 0; i < refs.size(); ++i) {
        const double v = std::max(0.0, refs[i].v_ref);
        ay_values.push_back(v * v * refs[i].kappa);
        if (i + 1 < refs.size()) {
            const double v_next = std::max(0.0, refs[i + 1].v_ref);
            const double ax = (v_next - v) / dt_safe;
            ax_values.push_back(ax);
            if (!has_first_ax) {
                first_ax = ax;
                has_first_ax = true;
            }
            if (has_prev_ax) {
                const double jerk = (ax - prev_ax) / dt_safe;
                jerk_values.push_back(jerk);
                if (jerk_values.size() == 1) {
                    first_jerk = jerk;
                }
            }
            prev_ax = ax;
            has_prev_ax = true;
        }
    }

    const ReferencePoint& ref0 = refs.front();
    publishFloat(ref_v_ref_pub_, ref0.v_ref);
    if (ref_v_ref_horizon_pub_.getNumSubscribers() > 0 ||
        ref_s_horizon_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32MultiArray v_msg;
        std_msgs::Float32MultiArray s_msg;
        v_msg.data.reserve(refs.size());
        s_msg.data.reserve(refs.size());
        for (const auto& ref : refs) {
            v_msg.data.push_back(static_cast<float>(std::max(0.0, ref.v_ref)));
            s_msg.data.push_back(static_cast<float>(ref.s));
        }
        if (ref_v_ref_horizon_pub_.getNumSubscribers() > 0) {
            ref_v_ref_horizon_pub_.publish(v_msg);
        }
        if (ref_s_horizon_pub_.getNumSubscribers() > 0) {
            ref_s_horizon_pub_.publish(s_msg);
        }
    }
    publishFloat(ref_v_path_pub_, ref0.v_path);
    publishFloat(ref_kappa_pub_, ref0.kappa);
    publishFloat(ref_s_pub_, ref0.s);
    publishFloat(ref_implied_ax_pub_, has_first_ax ? first_ax : 0.0);
    publishFloat(ref_implied_ay_pub_, ay_values.empty() ? 0.0 : ay_values.front());
    publishFloat(ref_implied_jerk_pub_, first_jerk);
    publishFloat(ref_implied_ax_abs_p95_pub_, absP95(ax_values));
    publishFloat(ref_implied_ay_abs_p95_pub_, absP95(ay_values));
    publishFloat(ref_implied_jerk_abs_p95_pub_, absP95(jerk_values));
}

void DiagnosticsPublisher::publishSloshDebug(const SloshDebugData& data) {
    publishInt(slosh_episode_id_pub_, data.episode_id);
    publishFloat(slosh_height_pred_max_pub_, data.predicted_height_max);
    publishFloat(slosh_q_slosh_eta_pub_, data.q_slosh_eta);
    publishInt(slosh_constraint_active_pub_, data.constraint_active);
    publishFloat(slosh_v_des_eff_pub_, data.v_des_eff);
    publishFloat(ref_v_des_raw_pub_, data.v_des_raw);
    publishFloat(ref_v_des_target_pub_, data.v_des_target);
    publishFloat(ref_v_des_eff_pub_, data.v_des_eff);
    publishInt(ref_v_des_rate_limited_pub_, data.v_des_rate_limited_active);
    publishFloat(slosh_omega_est_used_pub_, data.feedback.omega);
    publishFloat(slosh_imu_omega_z_filtered_pub_,
                 data.feedback.has_imu ? data.feedback.imu_omega_z_filtered : 0.0);
    publishFloat(slosh_imu_ay_bias_pub_,
                 data.imu_ay_bias_compensation_enable ? data.feedback.imu_ay_bias : 0.0);
    publishFloat(slosh_imu_ay_filtered_pub_,
                 data.feedback.has_imu ? data.feedback.imu_ay_filtered : 0.0);
    publishInt(slosh_imu_ay_bias_ready_pub_,
               (data.imu_ay_bias_compensation_enable && data.feedback.imu_ay_bias_ready) ? 1 : 0);

    const double eta_norm =
        std::hypot(static_cast<double>(data.slosh_state(0)),
                   static_cast<double>(data.slosh_state(2)));
    const double eta_dot_norm =
        std::hypot(static_cast<double>(data.slosh_state(1)),
                   static_cast<double>(data.slosh_state(3)));
    const double modal_energy =
        data.omega_n * data.omega_n * eta_norm * eta_norm + eta_dot_norm * eta_dot_norm;
    const double modal_energy_norm = std::sqrt(std::max(0.0, modal_energy));

    if (slosh_state_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32MultiArray msg;
        msg.data.resize(4);
        msg.data[0] = static_cast<float>(data.slosh_state(0));
        msg.data[1] = static_cast<float>(data.slosh_state(1));
        msg.data[2] = static_cast<float>(data.slosh_state(2));
        msg.data[3] = static_cast<float>(data.slosh_state(3));
        slosh_state_pub_.publish(msg);
    }

    publishFloat(slosh_eta_norm_pub_, eta_norm);
    publishFloat(slosh_eta_dot_norm_pub_, eta_dot_norm);
    publishFloat(slosh_modal_energy_pub_, modal_energy);
    publishFloat(slosh_modal_energy_norm_pub_, modal_energy_norm);
    publishFloat(slosh_excitation_ay_abs_pub_, std::abs(data.feedback.ay));
    publishFloat(slosh_excitation_alpha_abs_pub_, std::abs(data.feedback.alpha));
    publishFloat(slosh_height_pub_, data.slosh_enabled ? data.slosh_height : 0.0);

    if (data.publish_solver_debug) {
        publishFloat(mpc_solve_ms_pub_, data.solve_time_ms);
        publishInt(mpc_status_val_pub_, data.solve_ok ? 1 : 0);
    }

    publishFloat(slosh_ax_est_pub_, data.feedback.ax);
    publishFloat(slosh_ay_est_pub_, data.feedback.ay);
    publishFloat(slosh_alpha_est_pub_, data.feedback.alpha);
}

}  // namespace scout_local_planner
