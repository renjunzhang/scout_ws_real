/**
 * @file local_planner_ros.cpp
 * @brief ROS 接口实现
 */

#include "scout_local_planner/local_planner_ros.h"
#include "scout_local_planner/diff_drive_model.h"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <limits>
#include <algorithm>

namespace scout_local_planner {

namespace {

double computeTrimmedMean(std::vector<double> samples, double trim_ratio) {
    if (samples.empty()) {
        return 0.0;
    }

    std::sort(samples.begin(), samples.end());

    const double clamped_trim = std::max(0.0, std::min(0.49, trim_ratio));
    std::size_t trim_count = static_cast<std::size_t>(
        std::floor(static_cast<double>(samples.size()) * clamped_trim));
    if (trim_count * 2 >= samples.size()) {
        trim_count = 0;
    }

    const std::size_t begin = trim_count;
    const std::size_t end = samples.size() - trim_count;
    if (begin >= end) {
        return samples[samples.size() / 2];
    }

    double sum = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        sum += samples[i];
    }
    return sum / static_cast<double>(end - begin);
}

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

void updateNormalizedSloshWeights(MPCParams& params, double h_coeff, double omega_n) {
    const double h_ref = std::max(1e-4, params.slosh_height_ref);
    params.Q_slosh_eta = params.Q_slosh * h_coeff * h_coeff / (h_ref * h_ref);

    if (params.slosh_eta_dot_ratio > 0.0) {
        params.Q_slosh_eta_dot = omega_n > 1e-6
            ? params.slosh_eta_dot_ratio * params.Q_slosh_eta / (omega_n * omega_n)
            : 0.0;
    }
}

// 运动学终点速度包络。
//
// 性质:
//   - d 不在 terminal phase (d >= terminal_slowdown_distance) 或不可用: 返回 +inf (不约束)
//   - capture 前: 先把速度压到 terminal approach cap, 避免高速撞进 capture
//   - capture 后: 低速靠近 goal_tol, 到 goal_tol 才自然 enforce 停车
//   - 单调连续, 不需要 spatial smoothing (temporal rate limit 由 v_des_rate_limit_ 负责)
//
// 同一函数同时用于:
//   pre-MPC v_des cap (告知 MPC 正确 reference)
//   post-MPC v_cmd hard clamp (兜底, 防 MPC cost balance 输出超包络)
double computeTerminalVelocityEnvelope(
    double goal_dist,
    double terminal_slowdown_distance,
    double v_max_terminal,
    double goal_tol,
    double a_brake,
    bool capture_stop_enable,
    double capture_stop_distance,
    double terminal_approach_v_cap,
    bool goal_stop_pending) {
    if (!std::isfinite(goal_dist)) {
        return std::numeric_limits<double>::infinity();
    }
    const double a = std::max(1e-6, a_brake);
    const double v_cap = std::max(0.0, v_max_terminal);
    const double approach_cap = terminal_approach_v_cap > 1e-6
        ? terminal_approach_v_cap
        : v_cap;

    if (goal_stop_pending) {
        const double remain = std::max(0.0, goal_dist - goal_tol);
        const double v_kinematic = std::sqrt(2.0 * a * remain);
        return std::min(approach_cap, v_kinematic);
    }

    if (goal_dist >= terminal_slowdown_distance) {
        return std::numeric_limits<double>::infinity();
    }

    if (capture_stop_enable && capture_stop_distance > goal_tol + 1e-3) {
        const double remain_to_capture = std::max(0.0, goal_dist - capture_stop_distance);
        const double v_kinematic =
            std::sqrt(approach_cap * approach_cap + 2.0 * a * remain_to_capture);
        return std::min(v_cap, v_kinematic);
    }

    const double remain = std::max(0.0, goal_dist - goal_tol);
    const double v_kinematic = std::sqrt(2.0 * a * remain);
    return std::min(v_cap, v_kinematic);
}

double limitRate(double target, double current, double rate_limit, double dt) {
    if (!std::isfinite(target) || !std::isfinite(current) ||
        rate_limit <= 1e-6 || dt <= 1e-6) {
        return target;
    }
    const double max_delta = rate_limit * dt;
    return std::max(current - max_delta, std::min(current + max_delta, target));
}

double limitRateAsymmetric(
    double target,
    double current,
    double accel_limit,
    double decel_limit,
    double dt) {
    if (!std::isfinite(target) || !std::isfinite(current) || dt <= 1e-6) {
        return target;
    }
    const double up = std::max(0.0, accel_limit) * dt;
    const double down = std::max(0.0, decel_limit) * dt;
    if (target >= current) {
        return current + std::min(target - current, up);
    }
    return current - std::min(current - target, down);
}

}  // namespace

LocalPlannerROS::LocalPlannerROS() = default;
LocalPlannerROS::~LocalPlannerROS() = default;

bool LocalPlannerROS::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    nh_ = nh;
    
    // 加载参数
    loadParameters(pnh);
    
    // 初始化 TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 初始化路径处理器
    path_handler_.setParams(path_params_);
    path_handler_.setTFBuffer(tf_buffer_);
    
    // 初始化 MPC 求解器
    if (!mpc_solver_.initialize(mpc_params_, vehicle_params_)) {
        ROS_ERROR("[LocalPlannerROS] Failed to initialize MPC solver");
        return false;
    }

    // 初始化液体晃动集成 (P0-A)
    if (slosh_integration_.configure(slosh_params_)) {
        slosh_enabled_ = true;
        // 注入到动力学模型（需要 dynamic_pointer_cast 到 DiffDriveModel）
        auto diff_model = std::dynamic_pointer_cast<DiffDriveModel>(
            mpc_solver_.getDynamicsModel());
        if (diff_model) {
            diff_model->setSloshIntegration(&slosh_integration_);

            double h_coeff = slosh_integration_.getModalParams().height_coeff;
            const double omega_n = slosh_integration_.getModalParams().omega_n;
            updateNormalizedSloshWeights(mpc_params_, h_coeff, omega_n);

            if (mpc_params_.enable_slosh_box_constraint) {
                if (h_coeff <= 1e-9) {
                    ROS_WARN("[LocalPlannerROS] height_coeff too small, disabling slosh box constraint");
                    mpc_params_.enable_slosh_box_constraint = false;
                    mpc_params_.slosh_eta_bar = 0.0;
                } else {
                    const double omega_budget = std::max(0.0, vehicle_params_.omega_max);
                    double eta_parabola_budget = 0.0;
                    if (slosh_params_.use_parabola_term) {
                        const double R = slosh_params_.container_radius;
                        eta_parabola_budget = (R * R * omega_budget * omega_budget) / (4.0 * 9.81);
                    }

                    const double eta_modal_budget =
                        std::max(0.0, mpc_params_.slosh_height_max - eta_parabola_budget);
                    const double denom = h_coeff * std::sqrt(2.0);
                    mpc_params_.slosh_eta_bar = denom > 1e-9 ? eta_modal_budget / denom : 0.0;

                    if (mpc_params_.slosh_eta_bar <= 1e-9) {
                        ROS_WARN("[LocalPlannerROS] Modal budget too small after parabola reservation, disabling slosh box constraint");
                        mpc_params_.enable_slosh_box_constraint = false;
                        mpc_params_.slosh_eta_bar = 0.0;
                    } else {
                        ROS_INFO("[LocalPlannerROS] Slosh box constraint enabled: eta_bar=%.5f (height_max=%.4f, parabola_budget=%.4f)",
                                 mpc_params_.slosh_eta_bar,
                                 mpc_params_.slosh_height_max,
                                 eta_parabola_budget);
                    }
                }
            } else {
                mpc_params_.slosh_eta_bar = 0.0;
            }

            // 将更新后的参数同步到求解器（CostFunction 会用到 Q_slosh_eta）
            mpc_solver_.setMPCParams(mpc_params_);

            ROS_INFO("[LocalPlannerROS] Slosh integration enabled (Q_slosh=%.2f, h_ref=%.4f, eta_dot_ratio=%.3f, preview_factor=%.3f, h_coeff=%.4f, omega_n=%.4f, Q_slosh_eta=%.4f, Q_slosh_eta_dot=%.4f)",
                     mpc_params_.Q_slosh,
                     mpc_params_.slosh_height_ref,
                     mpc_params_.slosh_eta_dot_ratio,
                     mpc_params_.slosh_preview_factor,
                     h_coeff,
                     omega_n,
                     mpc_params_.Q_slosh_eta,
                     mpc_params_.Q_slosh_eta_dot);

        } else {
            slosh_enabled_ = false;
            ROS_WARN("[LocalPlannerROS] DiffDriveModel cast failed, slosh disabled");
        }
    } else {
        slosh_enabled_ = false;
        ROS_WARN("[LocalPlannerROS] Slosh integration configure failed, running without slosh");
    }
    
    // 订阅者
    global_path_sub_ = nh_.subscribe("global_path", 1, 
                                      &LocalPlannerROS::globalPathCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, 
                               &LocalPlannerROS::odomCallback, this);
    if (use_imu_lateral_accel_ || use_imu_yaw_rate_ || use_imu_alpha_z_) {
        imu_sub_ = nh_.subscribe(imu_topic_, 10,
                                 &LocalPlannerROS::imuCallback, this);
        ROS_INFO("[LocalPlannerROS] IMU interface enabled: topic=%s, ay=%s, omega_z=%s, alpha_z=%s",
                 imu_topic_.c_str(),
                 use_imu_lateral_accel_ ? "on" : "off",
                 use_imu_yaw_rate_ ? "on" : "off",
                 use_imu_alpha_z_ ? "on" : "off");
        ROS_INFO("[LocalPlannerROS] IMU ay bias compensation: %s (init=%.2fs, |v|<%.3f, |omega|<%.3f, min_samples=%d)",
                 imu_ay_bias_compensation_enable_ ? "on" : "off",
                 imu_ay_bias_init_duration_,
                 imu_ay_bias_static_v_max_,
                 imu_ay_bias_static_omega_max_,
                 imu_ay_bias_min_samples_);
        ROS_INFO("[LocalPlannerROS] IMU ay bias estimator: first_static_only, ema_alpha=%.2f, trim_ratio=%.2f",
                 imu_ay_bias_estimator_alpha_,
                 imu_ay_bias_trim_ratio_);
    }
    
    // 发布者
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("local_path", 1);
    reference_path_pub_ = nh_.advertise<nav_msgs::Path>("mpc/reference_path", 1);
    if (path_params_.publish_smoothed_path) {
        smoothed_path_pub_ = nh_.advertise<nav_msgs::Path>(
            path_params_.smoothed_path_topic, 1);
    }
    status_pub_ = nh_.advertise<std_msgs::String>("mpc_status", 1);

    // slosh 调试发布者
    slosh_state_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("slosh/state", 1);
    slosh_height_pub_ = nh_.advertise<std_msgs::Float32>("slosh/height", 1);
    slosh_ax_est_pub_ = nh_.advertise<std_msgs::Float32>("slosh/ax_est", 1);
    slosh_ay_est_pub_ = nh_.advertise<std_msgs::Float32>("slosh/ay_est", 1);
    slosh_alpha_est_pub_ = nh_.advertise<std_msgs::Float32>("slosh/alpha_est", 1);
    slosh_episode_id_pub_ = nh_.advertise<std_msgs::Int32>("slosh/episode_id", 1);
    slosh_height_pred_max_pub_ = nh_.advertise<std_msgs::Float32>("slosh/height_pred_max", 1);
    slosh_q_slosh_eta_pub_ = nh_.advertise<std_msgs::Float32>("slosh/q_slosh_eta", 1);
    slosh_constraint_active_pub_ = nh_.advertise<std_msgs::Int32>("slosh/constraint_active", 1);
    slosh_v_des_eff_pub_ = nh_.advertise<std_msgs::Float32>("slosh/v_des_eff", 1);
    slosh_omega_est_used_pub_ = nh_.advertise<std_msgs::Float32>("slosh/omega_est_used", 1);
    slosh_imu_omega_z_filtered_pub_ = nh_.advertise<std_msgs::Float32>("slosh/imu_omega_z_filtered", 1);
    slosh_imu_ay_bias_pub_ = nh_.advertise<std_msgs::Float32>("slosh/imu_ay_bias", 1);
    slosh_imu_ay_filtered_pub_ = nh_.advertise<std_msgs::Float32>("slosh/imu_ay_filtered", 1);
    slosh_imu_ay_bias_ready_pub_ = nh_.advertise<std_msgs::Int32>("slosh/imu_ay_bias_ready", 1);
    slosh_eta_norm_pub_ = nh_.advertise<std_msgs::Float32>("slosh/eta_norm", 1);
    slosh_eta_dot_norm_pub_ = nh_.advertise<std_msgs::Float32>("slosh/eta_dot_norm", 1);
    slosh_modal_energy_pub_ = nh_.advertise<std_msgs::Float32>("slosh/modal_energy", 1);
    slosh_modal_energy_norm_pub_ = nh_.advertise<std_msgs::Float32>("slosh/modal_energy_norm", 1);
    slosh_excitation_ay_abs_pub_ = nh_.advertise<std_msgs::Float32>("slosh/excitation_ay_abs", 1);
    slosh_excitation_alpha_abs_pub_ = nh_.advertise<std_msgs::Float32>("slosh/excitation_alpha_abs", 1);
    slosh_settling_time_pub_ = nh_.advertise<std_msgs::Float32>("slosh/settling_time", 1, true);
    mpc_solve_ms_pub_ = nh_.advertise<std_msgs::Float32>("mpc/solve_ms", 1);
    mpc_status_val_pub_ = nh_.advertise<std_msgs::Int32>("mpc/status_val", 1);
    mpc_cost_breakdown_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("mpc/cost_breakdown", 1);
    mpc_slosh_horizon_summary_pub_ =
        nh_.advertise<std_msgs::Float32MultiArray>("mpc/slosh_horizon_summary", 1);
    terminal_mode_pub_ = nh_.advertise<std_msgs::String>("terminal/mode", 1);
    terminal_recovery_latched_pub_ = nh_.advertise<std_msgs::Int32>("terminal/recovery_latched", 1);
    terminal_goal_info_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("terminal/goal_info", 1);
    terminal_v_envelope_pub_ = nh_.advertise<std_msgs::Float32>("terminal/v_envelope", 1);
    terminal_envelope_active_pub_ = nh_.advertise<std_msgs::Int32>("terminal/envelope_active", 1);
    terminal_phase_active_pub_ = nh_.advertise<std_msgs::Int32>("terminal/phase_active", 1);
    terminal_cmd_v_pre_clamp_pub_ =
        nh_.advertise<std_msgs::Float32>("terminal/cmd_v_pre_clamp", 1);
    terminal_cmd_v_post_clamp_pub_ =
        nh_.advertise<std_msgs::Float32>("terminal/cmd_v_post_clamp", 1);
    profile_cap_active_pub_ = nh_.advertise<std_msgs::Int32>("profile_cap/active", 1);
    profile_cap_v_profile_pub_ = nh_.advertise<std_msgs::Float32>("profile_cap/v_profile", 1);
    profile_cap_cmd_v_pre_pub_ = nh_.advertise<std_msgs::Float32>("profile_cap/cmd_v_pre_cap", 1);
    profile_cap_cmd_v_post_pub_ = nh_.advertise<std_msgs::Float32>("profile_cap/cmd_v_post_cap", 1);
    profile_cap_implied_ax_pub_ = nh_.advertise<std_msgs::Float32>("profile_cap/implied_ax", 1);
    profile_cap_implied_jerk_pub_ = nh_.advertise<std_msgs::Float32>("profile_cap/implied_jerk", 1);
    ref_v_ref_pub_ = nh_.advertise<std_msgs::Float32>("reference/v_ref", 1);
    ref_v_ref_horizon_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("reference/v_ref_horizon", 1);
    ref_s_horizon_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("reference/s_horizon", 1);
    ref_v_des_raw_pub_ = nh_.advertise<std_msgs::Float32>("reference/v_des_raw", 1);
    ref_v_des_target_pub_ = nh_.advertise<std_msgs::Float32>("reference/v_des_target", 1);
    ref_v_des_eff_pub_ = nh_.advertise<std_msgs::Float32>("reference/v_des_eff", 1);
    ref_v_des_rate_limited_pub_ = nh_.advertise<std_msgs::Int32>("reference/v_des_rate_limited", 1);
    ref_v_path_pub_ = nh_.advertise<std_msgs::Float32>("reference/v_path", 1);
    ref_kappa_pub_ = nh_.advertise<std_msgs::Float32>("reference/kappa", 1);
    ref_s_pub_ = nh_.advertise<std_msgs::Float32>("reference/s", 1);
    ref_implied_ax_pub_ = nh_.advertise<std_msgs::Float32>("reference/implied_ax", 1);
    ref_implied_ay_pub_ = nh_.advertise<std_msgs::Float32>("reference/implied_ay", 1);
    ref_implied_jerk_pub_ = nh_.advertise<std_msgs::Float32>("reference/implied_jerk", 1);
    ref_implied_ax_abs_p95_pub_ = nh_.advertise<std_msgs::Float32>("reference/implied_ax_abs_p95", 1);
    ref_implied_ay_abs_p95_pub_ = nh_.advertise<std_msgs::Float32>("reference/implied_ay_abs_p95", 1);
    ref_implied_jerk_abs_p95_pub_ = nh_.advertise<std_msgs::Float32>("reference/implied_jerk_abs_p95", 1);
    
    // 控制定时器
    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_rate_),
        &LocalPlannerROS::controlLoop, this);
    
    ROS_INFO("[LocalPlannerROS] Initialized successfully");
    ROS_INFO("  - Control rate: %.1f Hz", control_rate_);
    ROS_INFO("  - MPC horizon: N=%d, dt=%.3f", mpc_params_.N, mpc_params_.dt);
    
    return true;
}

void LocalPlannerROS::run() {
    ros::spin();
}

void LocalPlannerROS::loadParameters(ros::NodeHandle& pnh) {
    // MPC 参数
    pnh.param("mpc/N", mpc_params_.N, 60);
    pnh.param("mpc/dt", mpc_params_.dt, 1.0 / 30.0);
    pnh.param("mpc/cmd_vel_lead_time", mpc_params_.cmd_vel_lead_time, -1.0);
    pnh.param("mpc/Q_el", mpc_params_.Q_el, 1.0);
    pnh.param("mpc/Q_ec", mpc_params_.Q_ec, 10.0);
    pnh.param("mpc/Q_etheta", mpc_params_.Q_etheta, 5.0);
    pnh.param("mpc/Q_v", mpc_params_.Q_v, 1.0);
    pnh.param("mpc/use_contour_lag", mpc_params_.use_contour_lag, false);
    pnh.param("mpc/Q_contour", mpc_params_.Q_contour, mpc_params_.Q_ec);
    pnh.param("mpc/Q_lag", mpc_params_.Q_lag, mpc_params_.Q_el);
    pnh.param("mpc/enable_omega_ff", mpc_params_.enable_omega_ff, false);
    pnh.param("mpc/Q_omega_ff", mpc_params_.Q_omega_ff, 0.0);
    pnh.param("mpc/terminal_factor_ec", mpc_params_.terminal_factor_ec, 1.0);
    pnh.param("mpc/terminal_factor_etheta", mpc_params_.terminal_factor_etheta, 1.0);
    pnh.param("mpc/terminal_factor_v", mpc_params_.terminal_factor_v, 1.0);
    pnh.param("mpc/R_a", mpc_params_.R_a, 1.0);
    pnh.param("mpc/R_omega", mpc_params_.R_omega, 0.1);
    pnh.param("mpc/R_da", mpc_params_.R_da, 0.1);
    pnh.param("mpc/R_domega", mpc_params_.R_domega, 0.1);
    pnh.param("mpc/constrain_omega_rate", mpc_params_.constrain_omega_rate, true);
    pnh.param("mpc/constrain_accel_rate", mpc_params_.constrain_accel_rate, false);
    pnh.param("mpc/terminal_ramp_steps", mpc_params_.terminal_ramp_steps, 1);
    pnh.param("mpc/Q_slosh", mpc_params_.Q_slosh, 0.0);
    pnh.param("mpc/slosh_height_ref", mpc_params_.slosh_height_ref, 0.005);
    pnh.param("mpc/slosh_eta_dot_ratio", mpc_params_.slosh_eta_dot_ratio, 0.3);
    pnh.param("mpc/slosh_preview_factor", mpc_params_.slosh_preview_factor, 0.0);
    pnh.param("mpc/Q_slosh_eta_dot", mpc_params_.Q_slosh_eta_dot, 0.0);
    pnh.param("mpc/terminal_factor_slosh_eta", mpc_params_.terminal_factor_slosh_eta, 0.0);
    pnh.param("mpc/terminal_factor_slosh_eta_dot", mpc_params_.terminal_factor_slosh_eta_dot, 0.0);
    pnh.param("mpc/slosh_height_max", mpc_params_.slosh_height_max, 0.05);
    pnh.param("mpc/enable_slosh_box_constraint", mpc_params_.enable_slosh_box_constraint, false);

    // 液体晃动模型参数
    pnh.param("slosh/container_radius", slosh_params_.container_radius, 0.0185);
    pnh.param("slosh/liquid_height", slosh_params_.liquid_height, 0.058);
    pnh.param("slosh/liquid_density", slosh_params_.liquid_density, 1000.0);
    pnh.param("slosh/damping_ratio", slosh_params_.damping_ratio, 0.05);
    pnh.param("slosh/mode_index", slosh_params_.mode_index, 1);
    pnh.param("slosh/offset_x", slosh_params_.offset_x, 0.0);
    pnh.param("slosh/offset_y", slosh_params_.offset_y, 0.0);
    pnh.param("slosh/use_parabola_term", slosh_params_.use_parabola_term, true);
    pnh.param("slosh/use_linear_model", slosh_params_.use_linear_model, true);
    slosh_params_.dt = mpc_params_.dt;  // 与 MPC 时间步长一致

    // 加速度估计 EMA 滤波系数
    pnh.param("slosh_estimator/accel_filter_alpha", accel_filter_alpha_, 0.3);
    pnh.param("slosh_estimator/use_imu_lateral_accel", use_imu_lateral_accel_, false);
    pnh.param("slosh_estimator/use_imu_yaw_rate", use_imu_yaw_rate_, true);
    pnh.param("slosh_estimator/use_imu_alpha_z", use_imu_alpha_z_, false);
    pnh.param("slosh_estimator/imu_topic", imu_topic_, std::string("/imu/data"));
    pnh.param("slosh_estimator/imu_filter_alpha", imu_filter_alpha_, 0.3);
    pnh.param("slosh_estimator/imu_ay_bias_compensation_enable",
              imu_ay_bias_compensation_enable_, true);
    pnh.param("slosh_estimator/imu_ay_bias_init_duration",
              imu_ay_bias_init_duration_, 3.0);
    pnh.param("slosh_estimator/imu_ay_bias_static_v_max",
              imu_ay_bias_static_v_max_, 0.03);
    pnh.param("slosh_estimator/imu_ay_bias_static_omega_max",
              imu_ay_bias_static_omega_max_, 0.03);
    pnh.param("slosh_estimator/imu_ay_bias_min_samples",
              imu_ay_bias_min_samples_, 100);
    pnh.param("slosh_estimator/imu_ay_bias_estimator_alpha",
              imu_ay_bias_estimator_alpha_, 0.15);
    pnh.param("slosh_estimator/imu_ay_bias_trim_ratio",
              imu_ay_bias_trim_ratio_, 0.10);
    pnh.param("slosh_estimator/imu_ay_scale",
              imu_ay_scale_, 1.0);

    // 车辆参数
    pnh.param("vehicle/v_max", vehicle_params_.v_max, 1.0);
    pnh.param("vehicle/v_min", vehicle_params_.v_min, -0.3);
    pnh.param("vehicle/omega_max", vehicle_params_.omega_max, 1.0);
    pnh.param("vehicle/a_max", vehicle_params_.a_max, 0.5);
    pnh.param("vehicle/alpha_max", vehicle_params_.alpha_max, 1.0);
    pnh.param("vehicle/j_max", vehicle_params_.j_max, 0.0);
    pnh.param("vehicle/track_width", vehicle_params_.track_width, 0.456);
    
    // 路径处理参数
    pnh.param("path_handler/lookahead_distance", path_params_.lookahead_distance, 1.0);
    pnh.param("path_handler/goal_tolerance", path_params_.goal_tolerance, 0.1);
    pnh.param("path_handler/yaw_tolerance", path_params_.yaw_tolerance, 0.1);
    pnh.param("path_handler/goal_reached_max_speed",
              path_params_.goal_reached_max_speed, 0.08);
    pnh.param("path_handler/goal_reached_max_omega",
              path_params_.goal_reached_max_omega, 0.15);
    pnh.param("path_handler/goal_capture_distance", path_params_.goal_capture_distance, 0.4);
    pnh.param("path_handler/goal_capture_min_speed", path_params_.goal_capture_min_speed, 0.08);
    pnh.param("path_handler/path_timeout", path_params_.path_timeout, 5.0);
    pnh.param("path_handler/window_back", path_params_.window_back, 2);
    pnh.param("path_handler/window_forward", path_params_.window_forward, 2);
    pnh.param("path_handler/s_jump_threshold", path_params_.s_jump_threshold, 0.5);
    pnh.param("path_handler/resample_spacing", path_params_.resample_spacing, 0.0);
    pnh.param("path_handler/max_lat_accel", path_params_.max_lat_accel, 0.0);
    pnh.param("path_handler/min_ref_speed", path_params_.min_ref_speed, 0.0);
    pnh.param("path_handler/time_parameterize", path_params_.time_parameterize, false);
    pnh.param("path_handler/speed_profile_ds", path_params_.speed_profile_ds, 0.05);
    pnh.param("path_handler/external_speed_profile_csv",
              path_params_.external_speed_profile_csv, std::string(""));
    pnh.param("path_handler/max_tan_accel", path_params_.max_tan_accel, 0.0);
    pnh.param("path_handler/max_tan_decel", path_params_.max_tan_decel, 0.0);
    pnh.param("path_handler/goal_speed", path_params_.goal_speed, 0.0);
    pnh.param("path_handler/use_bspline_smoothing", path_params_.use_bspline_smoothing, false);
    pnh.param("path_handler/bspline_samples_per_segment", path_params_.bspline_samples_per_segment, 8);
    pnh.param("path_handler/speed_profile_omega_max", path_params_.speed_profile_omega_max, 0.0);
    pnh.param("path_handler/speed_profile_alpha_max", path_params_.speed_profile_alpha_max, 0.0);
    pnh.param("path_handler/publish_smoothed_path",
              path_params_.publish_smoothed_path, false);
    pnh.param("path_handler/smoothed_path_topic",
              path_params_.smoothed_path_topic,
              std::string("global_path_smooth"));
    pnh.param("path_handler/smoothed_path_points",
              path_params_.smoothed_path_points, 80);
    
    // 其他参数
    pnh.param("control_rate", control_rate_, 30.0);
    pnh.param("base_frame", base_frame_, std::string("base_link"));
    pnh.param("map_frame", map_frame_, std::string("map"));
    pnh.param("verbose", verbose_, false);
    pnh.param("safety/infeasible_decel", infeasible_decel_, 1.0);
    pnh.param("safety/infeasible_omega_scale", infeasible_omega_scale_, 0.0);
    pnh.param("safety/infeasible_min_speed", infeasible_min_speed_, 0.0);
    pnh.param("safety/tracking_feasibility_guard_enable",
              tracking_feasibility_guard_enable_, true);
    pnh.param("safety/tracking_feas_fail_trigger_count",
              tracking_feas_fail_trigger_count_, 3);
    pnh.param("safety/tracking_feas_fail_strong_trigger_count",
              tracking_feas_fail_strong_trigger_count_, 6);
    pnh.param("safety/tracking_feas_release_success_count",
              tracking_feas_release_success_count_, 5);
    pnh.param("safety/tracking_feas_v_cap_mild",
              tracking_feas_v_cap_mild_, 0.5);
    pnh.param("safety/tracking_feas_v_cap_strong",
              tracking_feas_v_cap_strong_, 0.3);
    pnh.param("safety/tracking_reentry_v_cap",
              tracking_reentry_v_cap_, 0.6);
    pnh.param("safety/tracking_reentry_ramp_steps",
              tracking_reentry_ramp_steps_, 10);
    pnh.param("safety/tracking_curvature_speed_cap_enable",
              tracking_curvature_speed_cap_enable_, false);
    pnh.param("safety/tracking_curvature_preview_distance",
              tracking_curvature_preview_distance_, 1.5);
    pnh.param("safety/tracking_curvature_rate_preview_distance",
              tracking_curvature_rate_preview_distance_, 1.0);
    pnh.param("safety/tracking_curvature_min_speed",
              tracking_curvature_min_speed_, 0.25);
    pnh.param("safety/tracking_curvature_rate_min_speed",
              tracking_curvature_rate_min_speed_, 0.25);
    pnh.param("safety/tracking_curvature_rate_gain",
              tracking_curvature_rate_gain_, 1.0);
    pnh.param("v_des_rate_limit/enable", v_des_rate_limit_enable_, true);
    pnh.param("v_des_rate_limit/accel_limit", v_des_accel_limit_, 0.6);
    pnh.param("v_des_rate_limit/decel_limit", v_des_decel_limit_, 0.8);
    pnh.param("external_profile_execution_cap/enable",
              external_profile_execution_cap_enable_, false);
    pnh.param("external_profile_execution_cap/accel_limit",
              external_profile_execution_accel_limit_, 0.0);
    pnh.param("external_profile_execution_cap/decel_limit",
              external_profile_execution_decel_limit_, 0.0);
    pnh.param("external_profile_execution_cap/jerk_limit",
              external_profile_execution_jerk_limit_, 0.0);

    // 原地对齐模式
    pnh.param("heading_align/enable", heading_align_enable_, false);
    pnh.param("heading_align/enter_angle", heading_align_enter_, 0.8);
    pnh.param("heading_align/exit_angle", heading_align_exit_, 0.4);
    pnh.param("heading_align/omega_gain", heading_align_omega_gain_, 1.5);
    pnh.param("heading_align/max_omega", heading_align_max_omega_, 0.0);
    pnh.param("heading_align/start_distance", heading_align_start_dist_, 0.5);

    // 终点恢复（near-goal terminal recovery）
    pnh.param("terminal_recovery/enable", terminal_recovery_enable_, false);
    pnh.param("terminal_recovery/enter_distance", terminal_enter_distance_, 0.35);
    pnh.param("terminal_recovery/release_distance", terminal_release_distance_, 0.55);
    pnh.param("terminal_recovery/goal_behind_x", terminal_goal_behind_x_, -0.05);
    pnh.param("terminal_recovery/align_angle", terminal_align_angle_, 1.0);
    pnh.param("terminal_recovery/approach_slow_angle", terminal_approach_slow_angle_, 0.45);
    pnh.param("terminal_recovery/bearing_gain", terminal_bearing_gain_, 1.8);
    pnh.param("terminal_recovery/final_yaw_gain", terminal_final_yaw_gain_, 1.5);
    pnh.param("terminal_recovery/max_omega", terminal_max_omega_, 0.0);
    pnh.param("terminal_recovery/dist_gain", terminal_dist_gain_, 0.8);
    pnh.param("terminal_recovery/v_min", terminal_v_min_, 0.05);
    pnh.param("terminal_recovery/v_max", terminal_v_max_, 0.18);
    pnh.param("terminal_recovery/cmd_v_rate_limit", terminal_cmd_v_rate_limit_, 0.35);
    pnh.param("terminal_recovery/cmd_omega_rate_limit", terminal_cmd_omega_rate_limit_, 1.0);
    pnh.param("terminal_slowdown/enable", terminal_slowdown_enable_, true);
    pnh.param("terminal_slowdown/distance", terminal_slowdown_distance_, 1.20);
    pnh.param("terminal_slowdown/v_max", terminal_slowdown_v_max_, 0.18);
    pnh.param("terminal_slowdown/Q_v", terminal_slowdown_q_v_, 40.0);
    pnh.param("terminal_slowdown/terminal_factor_v",
              terminal_slowdown_terminal_factor_v_, 5.0);
    pnh.param("terminal_capture_stop/enable", terminal_capture_stop_enable_, true);
    pnh.param("terminal_capture_stop/distance", terminal_capture_stop_distance_, 0.70);

    // 终点残余晃动收敛（T2 settling）
    pnh.param("settling/enable", settling_enable_, false);
    pnh.param("settling/timeout_s", settling_timeout_s_, 3.0);
    pnh.param("settling/release_distance", settling_release_distance_, 0.45);
    pnh.param("settling/eta_tol", settling_eta_tol_, 0.0015);
    pnh.param("settling/eta_dot_tol", settling_eta_dot_tol_, 0.03);
    pnh.param("settling/speed_tol", settling_speed_tol_, 0.05);
    pnh.param("settling/omega_tol", settling_omega_tol_, 0.10);
    pnh.param("settling/required_steps", settling_required_steps_override_, 0);
    pnh.param("settling/Q_v", settling_q_v_, 30.0);
    pnh.param("settling/Q_eta", settling_q_eta_, 10.0);
    pnh.param("settling/eta_bar", settling_eta_bar_, 0.04);
    
    // cmd_vel 低通滤波参数
    pnh.param("filter/alpha_v", cmd_filter_alpha_v_, 0.3);
    pnh.param("filter/alpha_omega", cmd_filter_alpha_omega_, 0.4);
    pnh.param("filter/kappa_boost", cmd_filter_kappa_boost_, 0.5);
    pnh.param("experiment/reached_debug_duration", reached_debug_duration_, 5.0);

    // 路径相似性检测阈值
    pnh.param("path_handler/path_change_threshold",
              path_params_.path_change_threshold, 0.3);

    // 将 base_frame 传递给 path_handler
    path_params_.base_frame = base_frame_;

    // 将运动学约束桥接到速度剖面规划。
    // 仅在未显式配置 speed_profile_* 时才退回 vehicle 约束，避免规划层预算被强制绑死到 MPC 硬约束。
    if (path_params_.speed_profile_omega_max <= 1e-6) {
        path_params_.speed_profile_omega_max = vehicle_params_.omega_max;
    }
    if (path_params_.speed_profile_alpha_max <= 1e-6) {
        path_params_.speed_profile_alpha_max = vehicle_params_.alpha_max;
    }

}

void LocalPlannerROS::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (path_handler_.updateGlobalPath(*msg, vehicle_params_.v_max * 0.8)) {
        has_path_ = true;
        // 注：不在此处无条件 resetWarmStart()。
        // 路径是否显著变化由 PathHandler 的 reset_hint_ 标记，
        // controlLoop() 通过 consumeResetHint() 统一决定是否重置。
        
        if (state_ == PlannerState::IDLE || 
            state_ == PlannerState::REACHED ||
            state_ == PlannerState::ERROR) {
            transitionTo(PlannerState::TRACKING);
        }
    }
}

void LocalPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    current_v_ = msg->twist.twist.linear.x;
    current_omega_ = msg->twist.twist.angular.z;
    current_odom_time_ = msg->header.stamp;
    has_odom_ = true;
    
    // 更新位姿（从 odom 消息中提取）
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    
    // 更新 PathHandler 的机器人状态（关键！）
    path_handler_.updateRobotState(current_pose_, current_v_, current_omega_);
}

void LocalPlannerROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    const double ay_raw = msg->linear_acceleration.y;
    const double omega_z_raw = msg->angular_velocity.z;
    bool ay_bias_just_initialized = false;

    if (imu_ay_bias_compensation_enable_ &&
        !imu_ay_bias_ready_ &&
        !imu_ay_bias_window_closed_) {
        const bool static_for_bias =
            has_odom_ &&
            std::abs(current_v_) < imu_ay_bias_static_v_max_ &&
            std::abs(current_omega_) < imu_ay_bias_static_omega_max_;

        if (static_for_bias) {
            if (!imu_ay_bias_window_started_) {
                imu_ay_bias_window_started_ = true;
                imu_ay_bias_window_start_ = stamp;
                imu_ay_bias_window_ema_initialized_ = false;
                imu_ay_bias_window_ema_ = 0.0;
                imu_ay_bias_samples_.clear();
            }

            if (!imu_ay_bias_window_ema_initialized_) {
                imu_ay_bias_window_ema_ = ay_raw;
                imu_ay_bias_window_ema_initialized_ = true;
            } else {
                imu_ay_bias_window_ema_ =
                    imu_ay_bias_estimator_alpha_ * ay_raw +
                    (1.0 - imu_ay_bias_estimator_alpha_) * imu_ay_bias_window_ema_;
            }
            imu_ay_bias_samples_.push_back(imu_ay_bias_window_ema_);
        } else if (imu_ay_bias_window_started_) {
            const double elapsed = (stamp - imu_ay_bias_window_start_).toSec();
            const int min_samples = std::max(1, imu_ay_bias_min_samples_);
            const int sample_count = static_cast<int>(imu_ay_bias_samples_.size());

            if (elapsed >= imu_ay_bias_init_duration_ && sample_count >= min_samples) {
                imu_ay_bias_ = computeTrimmedMean(imu_ay_bias_samples_, imu_ay_bias_trim_ratio_);
                imu_ay_bias_ready_ = true;
                ay_bias_just_initialized = true;
                ROS_INFO("[LocalPlannerROS] IMU ay bias initialized from first static window: bias=%.5f, samples=%d, static_window=%.3fs, estimator=EMA(alpha=%.2f)+trimmed_mean(trim=%.2f)",
                         imu_ay_bias_,
                         sample_count,
                         elapsed,
                         imu_ay_bias_estimator_alpha_,
                         imu_ay_bias_trim_ratio_);
            } else {
                ROS_WARN("[LocalPlannerROS] IMU ay bias not initialized: first static window too short (elapsed=%.3fs, samples=%d, need>=%.3fs and >=%d). Bias compensation will stay disabled for this run.",
                         elapsed,
                         sample_count,
                         imu_ay_bias_init_duration_,
                         min_samples);
            }

            imu_ay_bias_window_closed_ = true;
            imu_ay_bias_window_started_ = false;
            imu_ay_bias_window_ema_initialized_ = false;
            imu_ay_bias_samples_.clear();
        } else if (has_odom_) {
            imu_ay_bias_window_closed_ = true;
            ROS_WARN("[LocalPlannerROS] IMU ay bias not initialized: robot moved before the first static window. Bias compensation will stay disabled for this run.");
        }
    }

    const double ay_bias =
        (imu_ay_bias_compensation_enable_ && imu_ay_bias_ready_) ? imu_ay_bias_ : 0.0;
    imu_ay_unbiased_ = (ay_raw - ay_bias) * imu_ay_scale_;

    if (!has_imu_) {
        imu_ay_filtered_ = imu_ay_unbiased_;
        imu_omega_z_filtered_ = omega_z_raw;
        imu_alpha_filtered_ = 0.0;
        prev_imu_omega_z_ = omega_z_raw;
        prev_imu_time_ = stamp;
        has_imu_ = true;
        has_prev_imu_ = true;
        return;
    }

    if (ay_bias_just_initialized) {
        imu_ay_filtered_ = imu_ay_unbiased_;
    } else {
        imu_ay_filtered_ =
            imu_filter_alpha_ * imu_ay_unbiased_ + (1.0 - imu_filter_alpha_) * imu_ay_filtered_;
    }
    imu_omega_z_filtered_ =
        imu_filter_alpha_ * omega_z_raw + (1.0 - imu_filter_alpha_) * imu_omega_z_filtered_;

    if (has_prev_imu_) {
        const double dt_imu = (stamp - prev_imu_time_).toSec();
        if (dt_imu > 1e-4 && dt_imu < 1.0) {
            const double alpha_raw = (omega_z_raw - prev_imu_omega_z_) / dt_imu;
            imu_alpha_filtered_ =
                imu_filter_alpha_ * alpha_raw + (1.0 - imu_filter_alpha_) * imu_alpha_filtered_;
        }
    }

    prev_imu_omega_z_ = omega_z_raw;
    prev_imu_time_ = stamp;
    has_imu_ = true;
    has_prev_imu_ = true;
}

void LocalPlannerROS::controlLoop(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 更新状态
    updateState();

    // 路径跳变/重规划提示：重置 warm-start
    if (path_handler_.consumeResetHint()) {
        resetWarmStart(true);
        terminal_recovery_latched_ = false;
        tracking_solve_fail_streak_ = 0;
        tracking_solve_success_streak_ = 0;
        tracking_feasibility_recovery_active_ = false;
        {
            int ramp = std::max(0, tracking_reentry_ramp_steps_);
            if (tracking_curvature_speed_cap_enable_ && path_params_.max_lat_accel > 0.0) {
                const double kappa_start = path_handler_.getMaxCurvatureAhead(
                    0.0, tracking_curvature_preview_distance_);
                if (kappa_start > 1e-4 &&
                    std::sqrt(path_params_.max_lat_accel / kappa_start) <
                        tracking_reentry_v_cap_) {
                    ramp *= 2;
                }
            }
            tracking_reentry_ramp_steps_left_ = ramp;
        }
    }

    if (state_ != PlannerState::TRACKING) {
        heading_align_active_ = false;
    }

    // slosh 估计与调试输出不应只局限于 TRACKING。
    // 到达终点后的残余晃动衰减也需要继续观测。
    updateSloshEstimate();

    terminal_goal_info_valid_ =
        path_handler_.getGoalInfo(terminal_goal_info_debug_) && terminal_goal_info_debug_.valid;
    if (!terminal_goal_info_valid_) {
        terminal_goal_info_debug_ = GoalInfo();
    }

    switch (state_) {
        case PlannerState::IDLE:
            terminal_mode_debug_ = "IDLE";
            break;
        case PlannerState::ERROR:
            terminal_mode_debug_ = "ERROR";
            break;
        case PlannerState::SETTLING:
            terminal_mode_debug_ = "SETTLING";
            break;
        case PlannerState::REACHED:
            terminal_mode_debug_ = "REACHED";
            break;
        case PlannerState::TRACKING:
        default:
            if (goal_stop_pending_) {
                terminal_mode_debug_ = "TERMINAL_MPC_STOP";
            } else if (terminal_recovery_latched_) {
                terminal_mode_debug_ = "TERMINAL_LATCHED";
            } else {
                terminal_mode_debug_ = "NONE";
            }
            break;
    }
    
    // 发布状态
    publishStatus();
    publishTerminalDebug();
    
    // 根据状态机执行
    switch (state_) {
        case PlannerState::IDLE:
        case PlannerState::ERROR:
            // 停止
            publishCmdVel(0.0, 0.0);
            publishSloshDebug(last_solve_time_ms_, last_solve_ok_, false);
            break;
            
        case PlannerState::REACHED:
            // 到达目标，停止
            publishCmdVel(0.0, 0.0);
            ROS_INFO_THROTTLE(5.0, "[LocalPlannerROS] Goal reached");
            if (reached_time_.isZero() ||
                (ros::Time::now() - reached_time_).toSec() <= reached_debug_duration_) {
                publishSloshDebug(last_solve_time_ms_, last_solve_ok_, false);
            }
            break;
            
        case PlannerState::SETTLING:
        case PlannerState::TRACKING:
            // 执行 MPC 控制
            {
                const bool settling_active = (state_ == PlannerState::SETTLING);
                GoalInfo goal_info;
                const bool has_goal_info = path_handler_.getGoalInfo(goal_info);
                const bool terminal_capture_stop_active =
                    !settling_active &&
                    terminal_capture_stop_enable_ &&
                    has_goal_info &&
                    goal_info.valid &&
                    std::isfinite(goal_info.dist) &&
                    goal_info.dist < terminal_capture_stop_distance_;

                if (terminal_capture_stop_active) {
                    goal_stop_pending_ = true;
                    terminal_recovery_latched_ = false;
                    heading_align_active_ = false;
                    terminal_mode_debug_ = "TERMINAL_MPC_STOP";
                }

                const bool terminal_recovery_allowed =
                    has_goal_info &&
                    goal_info.valid &&
                    (goal_info.position_reached || goal_info.dx < terminal_goal_behind_x_);
                if (!settling_active &&
                    terminal_recovery_enable_ &&
                    !goal_stop_pending_ &&
                    terminal_recovery_allowed) {
                    if (terminal_recovery_latched_) {
                        const bool should_release =
                            !has_goal_info ||
                            !goal_info.valid ||
                            !std::isfinite(goal_info.dist) ||
                            goal_info.dist > terminal_release_distance_;
                        if (should_release) {
                            terminal_recovery_latched_ = false;
                        }
                    } else if (has_goal_info &&
                               goal_info.valid &&
                               std::isfinite(goal_info.dist) &&
                               goal_info.dist < terminal_enter_distance_) {
                        terminal_recovery_latched_ = true;
                    }
                } else {
                    terminal_recovery_latched_ = false;
                }

                if (!settling_active &&
                    terminal_recovery_enable_ &&
                    terminal_recovery_latched_ &&
                    has_goal_info &&
                    goal_info.valid &&
                    !goal_stop_pending_) {
                    double term_v = 0.0;
                    double term_omega = 0.0;
                    TerminalMode term_mode = TerminalMode::NONE;
                    if (computeTerminalRecoveryCmd(goal_info, term_v, term_omega, term_mode)) {
                        const double prev_cmd_v = filtered_v_;
                        limitTerminalRecoveryCmd(term_v, term_omega);
                        heading_align_active_ = false;
                        switch (term_mode) {
                            case TerminalMode::ALIGN_TO_POINT:
                                terminal_mode_debug_ = "ALIGN_TO_POINT";
                                break;
                            case TerminalMode::APPROACH_POINT:
                                terminal_mode_debug_ = "APPROACH_POINT";
                                break;
                            case TerminalMode::ALIGN_FINAL_YAW:
                                terminal_mode_debug_ = "ALIGN_FINAL_YAW";
                                break;
                            case TerminalMode::NONE:
                            default:
                                terminal_mode_debug_ = "NONE";
                                break;
                        }
                        terminal_goal_info_debug_ = goal_info;
                        terminal_goal_info_valid_ = true;
                        publishCmdVel(term_v, term_omega);
                        const double dt_cmd = control_rate_ > 1e-3 ? 1.0 / control_rate_ : mpc_params_.dt;
                        last_control_(ControlIndex::A) =
                            (filtered_v_ - prev_cmd_v) / std::max(1e-6, dt_cmd);
                        last_control_(ControlIndex::OMEGA) = filtered_omega_;
                        publishTerminalDebug();
                        publishSloshDebug(last_solve_time_ms_, last_solve_ok_, false);

                        if (verbose_) {
                            ROS_INFO_THROTTLE(
                                0.5,
                                "[TerminalRecovery] mode=%d dist=%.3f bearing=%.3f yaw_err=%.3f cmd=(%.3f, %.3f)",
                                static_cast<int>(term_mode),
                                goal_info.dist,
                                goal_info.bearing,
                                goal_info.goal_yaw_err,
                                term_v,
                                term_omega);
                        }
                        return;
                    }
                }

                const double v_nominal = vehicle_params_.v_max * 0.8;
                MPCParams runtime_mpc_params = mpc_params_;
                if (settling_active) {
                    runtime_mpc_params.Q_el = 0.0;
                    runtime_mpc_params.Q_ec = 0.0;
                    runtime_mpc_params.Q_etheta = 0.0;
                    runtime_mpc_params.Q_contour = 0.0;
                    runtime_mpc_params.Q_lag = 0.0;
                    runtime_mpc_params.enable_omega_ff = false;
                    runtime_mpc_params.Q_omega_ff = 0.0;
                    runtime_mpc_params.Q_v = std::max(runtime_mpc_params.Q_v, settling_q_v_);
                    runtime_mpc_params.terminal_factor_ec = 1.0;
                    runtime_mpc_params.terminal_factor_etheta = 1.0;
                    runtime_mpc_params.terminal_factor_v =
                        std::max(1.0, runtime_mpc_params.terminal_factor_v);
                    runtime_mpc_params.Q_slosh =
                        std::max(runtime_mpc_params.Q_slosh, settling_q_eta_);
                    const double h_coeff = slosh_integration_.getModalParams().height_coeff;
                    updateNormalizedSloshWeights(
                        runtime_mpc_params,
                        h_coeff,
                        slosh_integration_.getModalParams().omega_n);
                    if (slosh_enabled_ &&
                        runtime_mpc_params.enable_slosh_box_constraint &&
                        h_coeff > 1e-9) {
                        const double denom = h_coeff * std::sqrt(2.0);
                        runtime_mpc_params.slosh_eta_bar = settling_eta_bar_ / denom;
                    }
                }
                // Terminal velocity envelope: pre-MPC reference cap.
                // 单一运动学包络函数:
                //   capture 前: 在 terminal_capture_stop_distance_ 附近压到 terminal approach cap
                //   capture 后: 低速靠近 goal_tolerance, 到 goal_tolerance 才压到 0
                // 注意 envelope_active 与 in_terminal_phase 不同:
                //   envelope_active: d 在 envelope 作用范围内 (kinematic 约束有限)
                //   in_terminal_phase: terminal-phase protections 应生效的范围。
                //     必须包含 goal_stop_pending_ 因为车冲过 goal 后 dist 可能再次 > 1.20m,
                //     此时 envelope 退回 +inf, 但 goal_stop_pending_ 仍 true,
                //     post-MPC 的 dx ≤ 0 强制 0 兜底必须仍然生效。
                // v_des temporal 平滑由 v_des_rate_limit_ 负责, envelope 本身不再做 spatial smoothing。
                const double goal_dist_now = path_handler_.getGoalDistance();
                double a_brake =
                    path_params_.max_tan_decel > 1e-6 ? path_params_.max_tan_decel
                                                       : vehicle_params_.a_max;
                if (v_des_rate_limit_enable_ && v_des_decel_limit_ > 1e-6) {
                    a_brake = std::min(a_brake, v_des_decel_limit_);
                }
                const double terminal_approach_v_cap =
                    terminal_v_max_ > 1e-6 ? terminal_v_max_
                                           : std::max(0.0, terminal_slowdown_v_max_);
                const double v_terminal_envelope = terminal_slowdown_enable_
                    ? computeTerminalVelocityEnvelope(
                        goal_dist_now,
                        terminal_slowdown_distance_,
                        terminal_slowdown_v_max_,
                        path_params_.goal_tolerance,
                        a_brake,
                        terminal_capture_stop_enable_,
                        terminal_capture_stop_distance_,
                        terminal_approach_v_cap,
                        goal_stop_pending_)
                    : std::numeric_limits<double>::infinity();
                const bool envelope_active = std::isfinite(v_terminal_envelope);
                const bool in_terminal_phase = goal_stop_pending_ || envelope_active;
                last_terminal_v_envelope_ = v_terminal_envelope;
                last_terminal_envelope_active_ = envelope_active ? 1 : 0;
                last_terminal_phase_active_ = in_terminal_phase ? 1 : 0;

                if (in_terminal_phase) {
                    runtime_mpc_params.Q_v =
                        std::max(runtime_mpc_params.Q_v, terminal_slowdown_q_v_);
                    runtime_mpc_params.terminal_factor_v =
                        std::max(runtime_mpc_params.terminal_factor_v,
                                 terminal_slowdown_terminal_factor_v_);
                }

                const bool goal_position_reached_now =
                    has_goal_info && goal_info.valid && goal_info.position_reached;
                const bool goal_behind_now =
                    has_goal_info &&
                    goal_info.valid &&
                    std::isfinite(goal_info.dx) &&
                    goal_info.dx <= 0.0;
                const double v_des_cmd_raw =
                    settling_active ? 0.0 :
                    (goal_stop_pending_ && (goal_position_reached_now || goal_behind_now)) ? 0.0 :
                    goal_stop_pending_ ? terminal_approach_v_cap :
                    v_nominal;
                // goal_stop_pending_ 不再直接把 v_des_raw 砍成 0。
                // capture 内若位置还没到，MPC 仍低速 approach；真正到 goal_tol 或越过 goal 才停。
                double v_des_cmd = std::min(v_des_cmd_raw, v_terminal_envelope);

                mpc_solver_.setMPCParams(runtime_mpc_params);

                int reentry_steps_dbg = tracking_reentry_ramp_steps_left_;
                int tracking_fail_streak_dbg = tracking_solve_fail_streak_;
                int tracking_feas_active_dbg = tracking_feasibility_recovery_active_ ? 1 : 0;
                if (state_ == PlannerState::TRACKING &&
                    !goal_stop_pending_ &&
                    !settling_active &&
                    tracking_feasibility_guard_enable_) {
                    if (tracking_reentry_ramp_steps_left_ > 0) {
                        const int total_steps = std::max(1, tracking_reentry_ramp_steps_);
                        const int step_index =
                            std::max(0, total_steps - tracking_reentry_ramp_steps_left_);
                        const double alpha = static_cast<double>(step_index + 1) /
                                             static_cast<double>(total_steps);
                        const double ramp_floor = std::max(0.0, tracking_reentry_v_cap_);
                        const double ramp_cap =
                            ramp_floor + alpha * std::max(0.0, v_des_cmd_raw - ramp_floor);
                        v_des_cmd = std::min(v_des_cmd, ramp_cap);
                        --tracking_reentry_ramp_steps_left_;
                    }

                    if (tracking_solve_fail_streak_ >= tracking_feas_fail_strong_trigger_count_) {
                        v_des_cmd = std::min(v_des_cmd, tracking_feas_v_cap_strong_);
                        tracking_feasibility_recovery_active_ = true;
                    } else if (tracking_solve_fail_streak_ >= tracking_feas_fail_trigger_count_) {
                        v_des_cmd = std::min(v_des_cmd, tracking_feas_v_cap_mild_);
                        tracking_feasibility_recovery_active_ = true;
                    }

                    tracking_fail_streak_dbg = tracking_solve_fail_streak_;
                    tracking_feas_active_dbg = tracking_feasibility_recovery_active_ ? 1 : 0;
                }
                double kappa_preview_dbg = 0.0;
                double dkappa_preview_dbg = 0.0;
                if (state_ == PlannerState::TRACKING &&
                    !goal_stop_pending_ &&
                    !settling_active &&
                    tracking_curvature_speed_cap_enable_) {
                    double v_curve_cap = std::max(0.0, v_des_cmd);
                    if (path_params_.max_lat_accel > 1e-6) {
                        kappa_preview_dbg = path_handler_.getMaxCurvatureAhead(
                            path_params_.lookahead_distance,
                            tracking_curvature_preview_distance_);
                        if (kappa_preview_dbg > 1e-4) {
                            const double v_cap_kappa =
                                std::sqrt(std::max(0.0, path_params_.max_lat_accel /
                                                         (kappa_preview_dbg + 1e-9)));
                            // 规划层 omega 上限：使用 speed_profile_omega_max（与 v(s) 剖面一致）
                            // 与 QP 硬约束 vehicle_params_.omega_max 解耦，避免双重压速
                            const double v_cap_omega =
                                (path_params_.speed_profile_omega_max > 1e-3)
                                    ? path_params_.speed_profile_omega_max / (kappa_preview_dbg + 1e-9)
                                    : v_cap_kappa;
                            // 几何综合限速：取横向加速度 cap 与 omega_max cap 的较小值
                            const double v_cap_geom = std::min(v_cap_kappa, v_cap_omega);
                            // 若几何限速低于 min_speed，说明路径确实无法以 min_speed 行驶，降速优先
                            const double v_floor = std::min(tracking_curvature_min_speed_, v_cap_geom);
                            v_curve_cap = std::min(v_curve_cap, std::max(v_floor, v_cap_geom));
                        }
                    }
                    if (vehicle_params_.alpha_max > 1e-6) {
                        dkappa_preview_dbg = path_handler_.getMaxCurvatureRateAhead(
                            path_params_.lookahead_distance,
                            tracking_curvature_rate_preview_distance_);
                        if (dkappa_preview_dbg > 1e-4) {
                            const double v_cap_dkappa =
                                std::sqrt(std::max(0.0, tracking_curvature_rate_gain_ *
                                                         vehicle_params_.alpha_max /
                                                         (dkappa_preview_dbg + 1e-9)));
                            // alpha_max 硬约束：v² × dkappa ≤ alpha_max
                            // 该 cap 可低于 tracking_curvature_rate_min_speed_，
                            // 因路径几何（高 dkappa 段）确实要求低速
                            const double v_floor_dkappa =
                                std::min(tracking_curvature_rate_min_speed_, v_cap_dkappa);
                            v_curve_cap =
                                std::min(v_curve_cap,
                                         std::max(v_floor_dkappa, v_cap_dkappa));
                        }
                    }
                    v_des_cmd = std::min(v_des_cmd, v_curve_cap);
                }

                const double v_des_cmd_capped = v_des_cmd;
                double v_des_target = v_des_cmd_capped;

                // 对所有执行层 v_des 做变化率限制，避免速度参考突跳制造纵向 ax 脉冲。
                {
                    last_v_des_raw_ = v_des_cmd_raw;
                    last_v_des_target_ = v_des_target;
                    last_v_des_rate_limited_active_ = 0;

                    double v_des_eff = v_des_target;
                    if (v_des_rate_limit_enable_ && !settling_active) {
                        const bool terminal_stop_target =
                            goal_stop_pending_ || v_des_target <= 1e-6;
                        const double prev_v_des =
                            (!terminal_stop_target && last_v_des_eff_ <= 1e-6)
                                ? std::max(0.0, current_v_)
                                : std::max(0.0, last_v_des_eff_);
                        const double accel_limit =
                            v_des_accel_limit_ > 1e-6 ? v_des_accel_limit_ :
                            (path_params_.max_tan_accel > 1e-6 ? path_params_.max_tan_accel :
                             std::max(1e-6, vehicle_params_.a_max));
                        const double decel_limit =
                            v_des_decel_limit_ > 1e-6 ? v_des_decel_limit_ :
                            (path_params_.max_tan_decel > 1e-6 ? path_params_.max_tan_decel :
                             accel_limit);
                        const double rate_dt =
                            control_rate_ > 1e-3 ? 1.0 / control_rate_ : mpc_params_.dt;
                        const double v_lo =
                            std::max(0.0, prev_v_des - decel_limit * rate_dt);
                        const double v_hi = prev_v_des + accel_limit * rate_dt;
                        v_des_eff = std::max(v_lo, std::min(v_hi, v_des_target));
                        if (std::abs(v_des_eff - v_des_target) > 1e-4) {
                            last_v_des_rate_limited_active_ = 1;
                        }
                    }

                    const double v_des_upper =
                        goal_stop_pending_ ? std::max(0.0, current_v_) : v_des_cmd_capped;
                    last_v_des_eff_ =
                        std::max(0.0, std::min(v_des_upper, v_des_eff));
                    v_des_cmd = last_v_des_eff_;
                }

                // 1. 获取参考点。v_des_cmd 已经是 rate-limited 执行层速度上限。
                std::vector<ReferencePoint> ref_points;
                if (!path_handler_.getReferencePoints(
                        mpc_params_.N, mpc_params_.dt,
                        v_des_cmd,
                        v_nominal,
                        ref_points)) {
                    ROS_WARN_THROTTLE(1.0, "[LocalPlannerROS] Failed to get reference points");
                    publishCmdVel(0.0, 0.0);
                    return;
                }

                publishReferenceExecutionDebug(ref_points);

                publishSmoothedPath();
                
                // 2. 获取 Frenet 误差
                FrenetState frenet;
                if (!path_handler_.getFrenetState(frenet)) {
                    ROS_WARN_THROTTLE(1.0, "[LocalPlannerROS] Failed to get Frenet state");
                    publishCmdVel(0.0, 0.0);
                    return;
                }

                const double s_progress_dbg = path_handler_.getGlobalProgress();
                if (tracking_reentry_ramp_steps_left_ > 0) {
                    const ReferencePoint& ref0_dbg = ref_points.front();
                    ROS_INFO_THROTTLE(
                        0.5,
                        "[LocalPlannerROS][Reentry] s=%.3f e_l=%.3f e_c=%.3f e_theta=%.3f "
                        "v=%.3f omega=%.3f u_prev=(%.3f, %.3f) ref0=(x=%.3f y=%.3f th=%.3f v=%.3f k=%.3f) ramp_left=%d",
                        std::isfinite(s_progress_dbg) ? s_progress_dbg : -1.0,
                        frenet.e_l,
                        frenet.e_c,
                        frenet.e_theta,
                        current_v_,
                        current_omega_,
                        last_control_(ControlIndex::A),
                        last_control_(ControlIndex::OMEGA),
                        ref0_dbg.x,
                        ref0_dbg.y,
                        ref0_dbg.theta_path,
                        ref0_dbg.v_ref,
                        ref0_dbg.kappa,
                        tracking_reentry_ramp_steps_left_);
                }

                // 2.1 原地对齐模式：只在起点附近生效
                bool allow_heading_align = heading_align_enable_;
                if (allow_heading_align) {
                    const double start_dist = std::max(0.0, heading_align_start_dist_);
                    const double s_progress = path_handler_.getGlobalProgress();
                    if (s_progress > start_dist) {
                        allow_heading_align = false;
                        heading_align_active_ = false;
                    }
                }

                // 航向误差过大时先原地转向（仅限起点）
                if (allow_heading_align) {
                    const double abs_theta = std::abs(frenet.e_theta);
                    if (!heading_align_active_ && abs_theta > heading_align_enter_) {
                        heading_align_active_ = true;
                    } else if (heading_align_active_ && abs_theta < heading_align_exit_) {
                        heading_align_active_ = false;
                    }
                } else {
                    heading_align_active_ = false;
                }

                if (heading_align_active_) {
                    const double max_omega = heading_align_max_omega_ > 1e-6
                        ? heading_align_max_omega_
                        : vehicle_params_.omega_max;
                    // e_theta = theta_robot - theta_path，需取负号使其朝路径方向收敛
                    double omega = -heading_align_omega_gain_ * frenet.e_theta;
                    omega = std::max(-max_omega, std::min(max_omega, omega));
                    publishCmdVel(0.0, omega);

                    if (verbose_) {
                        ROS_INFO_THROTTLE(0.5, "[Align] e_theta=%.3f, omega=%.3f", frenet.e_theta, omega);
                    }
                    return;
                }
                
                // 3. 构建当前状态（避免初始状态越界导致不可行）
                // 注意：ω 现在是控制量，不在状态中！
                auto clamp = [](double v, double lo, double hi) {
                    return std::max(lo, std::min(hi, v));
                };

                double v_clamped = clamp(current_v_, vehicle_params_.v_min, vehicle_params_.v_max);

                StateVector current_state;
                current_state.setZero();  // 初始化所有状态（包括晃动状态）
                current_state(StateIndex::E_L) = frenet.e_l;
                current_state(StateIndex::E_C) = frenet.e_c;
                current_state(StateIndex::E_THETA) = frenet.e_theta;
                current_state(StateIndex::V) = v_clamped;

                // 将晃动模型的实际状态写入 x0 (P0-A: 状态注入)
                if (slosh_enabled_) {
                    slosh_integration_.writeToAugmentedState(current_state);
                }
                
                // 4. 设置上一步控制量
                const ControlVector u_prev_for_cost = last_control_;
                mpc_solver_.setPreviousControl(last_control_);
                
                // 5. 求解 MPC
                MPCSolution solution = mpc_solver_.solve(current_state, ref_points);
                
                if (solution.success) {
                    const CostBreakdown cost_breakdown =
                        computeCostBreakdown(solution, ref_points, runtime_mpc_params, u_prev_for_cost);

                    // 6. 发布控制命令
                    // 注意：cmd_vel 是速度，不是加速度！
                    // Terminal phase 用 envelope 兜底: MPC 输出超过运动学停车包络时硬 clamp,
                    // 越过 goal (dx ≤ 0) 时强制 0。omega 不做 envelope (不是 overshoot 主因)。
                    double cmd_v_out = solution.v_cmd;
                    last_terminal_cmd_v_pre_clamp_ = solution.v_cmd;
                    if (in_terminal_phase) {
                        cmd_v_out = std::min<double>(cmd_v_out, v_terminal_envelope);
                        GoalInfo gi;
                        bool goal_behind = false;
                        if (path_handler_.getGoalInfo(gi) && gi.valid &&
                            std::isfinite(gi.dx) && gi.dx <= 0.0) {
                            goal_behind = true;
                            cmd_v_out = 0.0;
                        }
                        cmd_v_out = std::max(0.0, cmd_v_out);
                        if (!goal_behind) {
                            const double dt_cmd =
                                control_rate_ > 1e-3 ? 1.0 / control_rate_ : mpc_params_.dt;
                            const double terminal_decel = std::max(1e-6, a_brake);
                            cmd_v_out = limitRate(
                                cmd_v_out,
                                std::max(0.0, filtered_v_),
                                terminal_decel,
                                dt_cmd);
                        }
                    }
                    last_profile_cap_active_ = 0;
                    last_profile_cap_v_profile_ = std::numeric_limits<double>::quiet_NaN();
                    last_profile_cap_cmd_v_pre_ = cmd_v_out;
                    last_profile_cap_cmd_v_post_ = cmd_v_out;
                    last_profile_cap_implied_ax_ = std::numeric_limits<double>::quiet_NaN();
                    last_profile_cap_implied_jerk_ = std::numeric_limits<double>::quiet_NaN();
                    bool profile_cap_applied = false;
                    if (external_profile_execution_cap_enable_ &&
                        path_handler_.hasExternalSpeedProfile()) {
                        const double s_now = path_handler_.getGlobalProgress();
                        double profile_v = path_handler_.getSpeedAtS(s_now);
                        if (std::isfinite(s_now) && profile_v <= 1e-6) {
                            profile_v = std::max(
                                profile_v,
                                path_handler_.getSpeedAtS(
                                    s_now + std::max(0.02, path_params_.speed_profile_ds)));
                        }
                        if (std::isfinite(s_now) && std::isfinite(profile_v)) {
                            const double dt_cmd =
                                control_rate_ > 1e-3 ? 1.0 / control_rate_ : mpc_params_.dt;
                            const double prev_cmd_v = std::max(0.0, filtered_v_);
                            const double pre_cap = cmd_v_out;
                            const double target_v =
                                std::max(0.0, std::min(cmd_v_out, profile_v));
                            const double accel_limit =
                                external_profile_execution_accel_limit_ > 1e-6
                                    ? external_profile_execution_accel_limit_
                                    : (path_params_.max_tan_accel > 1e-6
                                           ? path_params_.max_tan_accel
                                           : std::max(1e-6, vehicle_params_.a_max));
                            const double decel_limit =
                                external_profile_execution_decel_limit_ > 1e-6
                                    ? external_profile_execution_decel_limit_
                                    : (path_params_.max_tan_decel > 1e-6
                                           ? path_params_.max_tan_decel
                                           : accel_limit);
                            cmd_v_out = limitRateAsymmetric(
                                target_v, prev_cmd_v, accel_limit, decel_limit, dt_cmd);

                            double implied_ax = (cmd_v_out - prev_cmd_v) / std::max(1e-6, dt_cmd);
                            double implied_jerk = std::numeric_limits<double>::quiet_NaN();
                            if (external_profile_execution_jerk_limit_ > 1e-6 &&
                                profile_cap_has_last_ax_) {
                                const double ax_limited = limitRate(
                                    implied_ax,
                                    profile_cap_last_ax_,
                                    external_profile_execution_jerk_limit_,
                                    dt_cmd);
                                cmd_v_out = std::max(0.0, prev_cmd_v + ax_limited * dt_cmd);
                                cmd_v_out = std::min(cmd_v_out, profile_v);
                                implied_ax = (cmd_v_out - prev_cmd_v) / std::max(1e-6, dt_cmd);
                            }
                            if (profile_cap_has_last_ax_) {
                                implied_jerk =
                                    (implied_ax - profile_cap_last_ax_) / std::max(1e-6, dt_cmd);
                            }
                            profile_cap_last_ax_ = implied_ax;
                            profile_cap_has_last_ax_ = true;

                            last_profile_cap_active_ =
                                (std::abs(pre_cap - cmd_v_out) > 1e-4 ||
                                 std::abs(pre_cap - target_v) > 1e-4) ? 1 : 0;
                            last_profile_cap_v_profile_ = profile_v;
                            last_profile_cap_cmd_v_pre_ = pre_cap;
                            last_profile_cap_cmd_v_post_ = cmd_v_out;
                            last_profile_cap_implied_ax_ = implied_ax;
                            last_profile_cap_implied_jerk_ = implied_jerk;
                            profile_cap_applied = true;
                        }
                    } else {
                        profile_cap_has_last_ax_ = false;
                    }
                    if (in_terminal_phase || profile_cap_applied) {
                        // publishCmdVel 内部还有 EMA。terminal/profile 输出层已经做过速度包络和
                        // 变化率限制，这里同步滤波状态，避免 EMA 再把上一帧高速带回输出。
                        filtered_v_ = cmd_v_out;
                    }
                    last_terminal_cmd_v_post_clamp_ = cmd_v_out;
                    publishCmdVel(cmd_v_out, solution.omega_cmd);
                    
                    // 保存控制量
                    last_control_ = solution.u_first;
                    
                    // 发布预测轨迹
                    publishLocalPath(solution.x_predicted, ref_points);
                    publishReferencePath(ref_points);
                    publishCostBreakdown(cost_breakdown);

                    // 发布 slosh 调试信息
                    last_solve_time_ms_ = solution.solve_time_ms;
                    last_solve_ok_ = true;
                    if (state_ == PlannerState::TRACKING) {
                        tracking_solve_fail_streak_ = 0;
                        ++tracking_solve_success_streak_;
                        if (tracking_feasibility_recovery_active_ &&
                            tracking_solve_success_streak_ >=
                                std::max(1, tracking_feas_release_success_count_)) {
                            tracking_feasibility_recovery_active_ = false;
                            tracking_solve_success_streak_ = 0;
                            ROS_INFO_THROTTLE(
                                1.0,
                                "[LocalPlannerROS] TRACKING feasibility recovery released after stable solves");
                        }
                    } else {
                        tracking_solve_fail_streak_ = 0;
                        tracking_solve_success_streak_ = 0;
                        tracking_feasibility_recovery_active_ = false;
                    }
                    last_predicted_height_max_ = computePredictedSloshHeightMax(solution);
                    publishSloshHorizonSummary(solution);
                    last_constraint_active_ =
                        (mpc_params_.slosh_height_max > 0.0 &&
                         last_predicted_height_max_ > mpc_params_.slosh_height_max) ? 1 : 0;
                    publishSloshDebug(solution.solve_time_ms, true);

                    if (verbose_) {
                        ROS_INFO_THROTTLE(0.5,
                            "[MPC] e_c=%.3f, e_theta=%.3f, v=%.3f, omega=%.3f, solve_time=%.1fms",
                            frenet.e_c, frenet.e_theta,
                            solution.v_cmd, solution.omega_cmd,
                            solution.solve_time_ms);
                    }
                } else {
                    ROS_WARN_THROTTLE(1.0, "[LocalPlannerROS] MPC solve failed: %s", 
                                      solution.status_msg.c_str());
                    {
                        const Eigen::Vector4d ss = slosh_enabled_
                            ? slosh_integration_.getSloshState()
                            : Eigen::Vector4d::Zero();
                        const double goal_dist_dbg = path_handler_.getGoalDistance();
                        const double s_progress_dbg = path_handler_.getGlobalProgress();
                        ROS_WARN_THROTTLE(
                            0.5,
                            "[LocalPlannerROS][SolveFail] state=%s term_mode=%s goal_dist=%.3f s=%.3f "
                            "v=%.3f omega=%.3f solve_ms=%.3f "
                            "e=(%.3f, %.3f, %.3f) u_prev=(%.3f, %.3f) ref0=(v=%.3f k=%.3f) "
                            "eta=(%.4f, %.4f) eta_dot=(%.4f, %.4f) "
                            "Qv=%.2f Qeta=%.2f eta_bar=%.4f "
                            "v_des=%.3f v_des_raw=%.3f fallback=%d fail_streak=%d feas_active=%d reentry=%d "
                            "kappa=%.4f dkappa=%.4f",
                            plannerStateToString(state_).c_str(),
                            terminal_mode_debug_.c_str(),
                            std::isfinite(goal_dist_dbg) ? goal_dist_dbg : -1.0,
                            std::isfinite(s_progress_dbg) ? s_progress_dbg : -1.0,
                            current_v_,
                            current_omega_,
                            solution.solve_time_ms,
                            frenet.e_l,
                            frenet.e_c,
                            frenet.e_theta,
                            last_control_(ControlIndex::A),
                            last_control_(ControlIndex::OMEGA),
                            ref_points.empty() ? 0.0 : ref_points.front().v_ref,
                            ref_points.empty() ? 0.0 : ref_points.front().kappa,
                            ss(0), ss(2), ss(1), ss(3),
                            runtime_mpc_params.Q_v,
                            runtime_mpc_params.Q_slosh_eta,
                            runtime_mpc_params.slosh_eta_bar,
                            last_v_des_eff_,
                            v_des_cmd_raw,
                            0,
                            tracking_fail_streak_dbg,
                            tracking_feas_active_dbg,
                            reentry_steps_dbg,
                            kappa_preview_dbg,
                            dkappa_preview_dbg);
                    }
                    if (state_ == PlannerState::TRACKING) {
                        ++tracking_solve_fail_streak_;
                        tracking_solve_success_streak_ = 0;
                        if (tracking_feasibility_guard_enable_ &&
                            tracking_solve_fail_streak_ >=
                                std::max(1, tracking_feas_fail_trigger_count_)) {
                            tracking_feasibility_recovery_active_ = true;
                            ROS_WARN_THROTTLE(
                                1.0,
                                "[LocalPlannerROS] TRACKING feasibility recovery active: fail_streak=%d v_cap=%.2f/%.2f",
                                tracking_solve_fail_streak_,
                                tracking_feas_v_cap_mild_,
                                tracking_feas_v_cap_strong_);
                        }
                    } else {
                        tracking_solve_fail_streak_ = 0;
                        tracking_solve_success_streak_ = 0;
                    }
                    double dt = control_rate_ > 1e-3 ? 1.0 / control_rate_ : mpc_params_.dt;
                    double v = current_v_;
                    double decel = std::max(0.0, infeasible_decel_);
                    if (std::abs(v) > 1e-3) {
                        double sign = v >= 0.0 ? 1.0 : -1.0;
                        v -= sign * decel * dt;
                        if (sign > 0.0) {
                            v = std::max(v, infeasible_min_speed_);
                            if (v < 0.0) v = 0.0;
                        } else {
                            v = std::min(v, -infeasible_min_speed_);
                            if (v > 0.0) v = 0.0;
                        }
                    } else {
                        v = 0.0;
                    }
                    double omega = current_omega_ * infeasible_omega_scale_;
                    last_terminal_cmd_v_pre_clamp_ = std::numeric_limits<double>::quiet_NaN();
                    last_terminal_cmd_v_post_clamp_ = v;
                    publishCmdVel(v, omega);
                    // 同步更新 last_control_，使下一周期的 alpha 约束基于实际命令值。
                    // 若不更新，u_prev.omega 会冻结在最后一次成功的解，导致
                    // |omega_0 - u_prev.omega| ≤ alpha_max·dt 持续给 omega_0 施加
                    // 与当前低速停止状态不符的下限，MPC QP 可能陷入永久不可行。
                    last_control_(ControlIndex::A) =
                        std::abs(current_v_) > 1e-3
                            ? (v - current_v_) / (control_rate_ > 1e-3 ? 1.0 / control_rate_ : mpc_params_.dt)
                            : 0.0;
                    last_control_(ControlIndex::OMEGA) = omega;
                    last_solve_time_ms_ = 0.0;
                    last_solve_ok_ = false;
                    last_predicted_height_max_ = std::numeric_limits<double>::quiet_NaN();
                    last_constraint_active_ = -1;
                    publishSloshDebug(0.0, false);
                }
            }
            break;
    }
}

bool LocalPlannerROS::computeTerminalRecoveryCmd(const GoalInfo& goal,
                                                 double& v_cmd,
                                                 double& omega_cmd,
                                                 TerminalMode& mode) const {
    v_cmd = 0.0;
    omega_cmd = 0.0;
    mode = TerminalMode::NONE;

    if (!goal.valid) {
        return false;
    }

    auto clamp = [](double x, double lo, double hi) {
        return std::max(lo, std::min(hi, x));
    };

    const double omega_max =
        terminal_max_omega_ > 1e-6 ? terminal_max_omega_ : vehicle_params_.omega_max;

    const bool goal_behind = goal.dx < terminal_goal_behind_x_;
    const double abs_bearing = std::abs(goal.bearing);

    // 位置已到但姿态还没到：不要退回 normal tracking。
    // 先把 goal 点几何关系对准，再补 final yaw。
    if (goal.position_reached && !goal.pose_reached) {
        const bool need_point_align = goal.dx <= 0.0 || abs_bearing >= 0.30;
        if (need_point_align) {
            mode = TerminalMode::ALIGN_TO_POINT;
            omega_cmd = clamp(terminal_bearing_gain_ * goal.bearing,
                              -omega_max, omega_max);
            return true;
        }

        if (goal.has_goal_yaw &&
            std::abs(goal.goal_yaw_err) > path_params_.yaw_tolerance) {
            mode = TerminalMode::ALIGN_FINAL_YAW;
            omega_cmd = clamp(terminal_final_yaw_gain_ * goal.goal_yaw_err,
                              -omega_max, omega_max);
            return true;
        }

        return false;
    }

    // goal 在车后，或当前 bearing 太大：先原地对准 goal 点。
    const bool bearing_large = abs_bearing > terminal_align_angle_;
    if (goal_behind || bearing_large) {
        mode = TerminalMode::ALIGN_TO_POINT;
        omega_cmd = clamp(terminal_bearing_gain_ * goal.bearing,
                          -omega_max, omega_max);
        return true;
    }

    // goal 在前方、距离还未达标：低速靠近。
    if (goal.dist > path_params_.goal_tolerance) {
        mode = TerminalMode::APPROACH_POINT;

        double v = clamp(terminal_dist_gain_ * goal.dist,
                         terminal_v_min_,
                         terminal_v_max_);
        if (abs_bearing > terminal_approach_slow_angle_) {
            v *= 0.4;
        }

        v_cmd = std::max(0.0, std::min(v, terminal_v_max_));
        omega_cmd = clamp(terminal_bearing_gain_ * goal.bearing,
                          -omega_max, omega_max);
        return true;
    }

    return false;
}

void LocalPlannerROS::limitTerminalRecoveryCmd(double& v_cmd, double& omega_cmd) const {
    const double dt = control_rate_ > 1e-3 ? 1.0 / control_rate_ : mpc_params_.dt;
    const double step_dt = std::max(1e-6, dt);

    auto limit_rate = [step_dt](double target, double current, double rate_limit) {
        if (!std::isfinite(target) || !std::isfinite(current) || rate_limit <= 1e-6) {
            return target;
        }
        const double max_delta = rate_limit * step_dt;
        return std::max(current - max_delta, std::min(current + max_delta, target));
    };

    v_cmd = limit_rate(v_cmd, filtered_v_, terminal_cmd_v_rate_limit_);
    omega_cmd = limit_rate(omega_cmd, filtered_omega_, terminal_cmd_omega_rate_limit_);
}

int LocalPlannerROS::computeSettlingRequiredSteps() const {
    if (settling_required_steps_override_ > 0) {
        return settling_required_steps_override_;
    }

    const double fallback_steps =
        std::ceil(0.5 / std::max(1e-6, mpc_params_.dt));
    if (!slosh_enabled_) {
        return std::max(1, static_cast<int>(fallback_steps));
    }

    const double omega_n = slosh_integration_.getModalParams().omega_n;
    if (omega_n <= 1e-6) {
        return std::max(1, static_cast<int>(fallback_steps));
    }

    return std::max(
        1,
        static_cast<int>(std::ceil((4.0 * M_PI / omega_n) / mpc_params_.dt)));
}

void LocalPlannerROS::publishSettlingTime(bool timeout) {
    const double duration_s = settling_enter_time_.isZero()
        ? settling_step_count_ * mpc_params_.dt
        : (ros::Time::now() - settling_enter_time_).toSec();
    if (!std::isfinite(duration_s) || duration_s < 0.0) {
        return;
    }

    std_msgs::Float32 msg;
    msg.data = static_cast<float>(duration_s);
    slosh_settling_time_pub_.publish(msg);

    ROS_INFO("[LocalPlannerROS] SETTLING finished by %s, settling_time=%.3fs",
             timeout ? "timeout" : "convergence",
             duration_s);
}

void LocalPlannerROS::updateState() {
    // 检查数据是否有效
    if (!has_odom_) {
        if (state_ != PlannerState::IDLE && state_ != PlannerState::ERROR) {
            transitionTo(PlannerState::ERROR);
            ROS_WARN("[LocalPlannerROS] No odometry data");
        }
        return;
    }
    
    if (!path_handler_.isPathValid()) {
        if (state_ == PlannerState::TRACKING || state_ == PlannerState::SETTLING) {
            transitionTo(PlannerState::ERROR);
            ROS_WARN("[LocalPlannerROS] Path invalid or timeout");
        }
        return;
    }
    
    if (state_ != PlannerState::TRACKING && state_ != PlannerState::SETTLING) {
        goal_stop_pending_ = false;
        terminal_recovery_latched_ = false;
        return;
    }

    GoalInfo goal_info;
    const bool has_goal_info = path_handler_.getGoalInfo(goal_info);
    const bool goal_position_reached =
        has_goal_info && goal_info.valid && goal_info.position_reached;
    const double goal_dist = path_handler_.getGoalDistance();
    const double goal_stop_release_dist =
        std::max({path_params_.goal_capture_distance,
                  path_params_.goal_tolerance + 0.15,
                  terminal_capture_stop_enable_ ? terminal_capture_stop_distance_ + 0.10 : 0.0});
    const bool terminal_capture_stop_reached =
        terminal_capture_stop_enable_ &&
        std::isfinite(goal_dist) &&
        goal_dist < terminal_capture_stop_distance_;

    if (state_ == PlannerState::SETTLING) {
        terminal_recovery_latched_ = false;
        goal_stop_pending_ = false;

        const bool should_release =
            !goal_position_reached &&
            (!std::isfinite(goal_dist) || goal_dist > settling_release_distance_);
        if (should_release) {
            transitionTo(PlannerState::TRACKING);
            return;
        }

        ++settling_step_count_;

        const Eigen::Vector4d ss = slosh_integration_.getSloshState();
        const bool slosh_small =
            std::abs(ss(0)) < settling_eta_tol_ &&
            std::abs(ss(1)) < settling_eta_dot_tol_ &&
            std::abs(ss(2)) < settling_eta_tol_ &&
            std::abs(ss(3)) < settling_eta_dot_tol_;
        const bool speed_low =
            std::abs(current_v_) < settling_speed_tol_ &&
            std::abs(current_omega_) < settling_omega_tol_;
        const bool enough_time = settling_step_count_ >= computeSettlingRequiredSteps();
        const bool timeout =
            settling_timeout_s_ > 0.0 &&
            settling_step_count_ * mpc_params_.dt >= settling_timeout_s_;

        if ((enough_time && slosh_small && speed_low) || timeout) {
            publishSettlingTime(timeout);
            transitionTo(PlannerState::REACHED);
            terminal_recovery_latched_ = false;
            resetWarmStart(false, false);
        }
        return;
    }

    if (goal_stop_pending_) {
        // 终点边界附近允许短暂滑出 pose gate，但不要立刻释放 pending stop。
        // 否则会出现“进圈一帧开始刹车 -> 滑出一帧又恢复巡航参考”的 limit cycle。
        // 一旦车体已经越过 goal（dx 为负），继续保持 pending stop，避免重新恢复巡航参考。
        const bool goal_behind =
            has_goal_info &&
            goal_info.valid &&
            std::isfinite(goal_info.dx) &&
            goal_info.dx < terminal_goal_behind_x_;
        const bool should_release =
            !std::isfinite(goal_dist) || (goal_dist > goal_stop_release_dist && !goal_behind);
        if (should_release) {
            goal_stop_pending_ = false;
        }
    }

    if (!goal_stop_pending_ && !goal_position_reached && !terminal_capture_stop_reached) {
        return;
    }

    if (goal_position_reached) {
        if (settling_enable_ && slosh_enabled_) {
            transitionTo(PlannerState::SETTLING);
            goal_stop_pending_ = false;
            terminal_recovery_latched_ = false;
            return;
        }
        goal_stop_pending_ = true;
    } else if (terminal_capture_stop_reached) {
        goal_stop_pending_ = true;
    }

    const bool speed_low =
        std::abs(current_v_) < path_params_.goal_reached_max_speed &&
        std::abs(current_omega_) < path_params_.goal_reached_max_omega;
    // REACHED 必须同时满足 speed-low 与 position-in-tolerance:
    // 仅看速度会让车在越过 goal 1m+ 后只要停住就误判 REACHED;
    // 加上 goal_position_reached 后, 车若过冲则状态保持 goal_stop_pending_ 直到 MPC 把
    // 车开回容差圈 (实际不会发生, 但状态机不会假阳性结束)。
    if (speed_low && goal_position_reached) {
        transitionTo(PlannerState::REACHED);
        goal_stop_pending_ = false;
        terminal_recovery_latched_ = false;
        // 到达终点后保留 slosh 内部状态一段时间，便于观测残余晃动衰减。
        resetWarmStart(false, false);
    }
}

void LocalPlannerROS::transitionTo(PlannerState new_state) {
    if (state_ != new_state) {
        if (new_state == PlannerState::TRACKING && state_ != PlannerState::TRACKING) {
            ++episode_id_;
            {
                // 基础 ramp；若起点曲率超出 reentry_v_cap 对应横向加速度，加倍保守
                int ramp = std::max(0, tracking_reentry_ramp_steps_);
                if (tracking_curvature_speed_cap_enable_ && path_params_.max_lat_accel > 0.0) {
                    const double kappa_start = path_handler_.getMaxCurvatureAhead(
                        0.0, tracking_curvature_preview_distance_);
                    if (kappa_start > 1e-4 &&
                        std::sqrt(path_params_.max_lat_accel / kappa_start) <
                            tracking_reentry_v_cap_) {
                        ramp *= 2;
                    }
                }
                tracking_reentry_ramp_steps_left_ = ramp;
            }
            tracking_solve_fail_streak_ = 0;
            tracking_solve_success_streak_ = 0;
            tracking_feasibility_recovery_active_ = false;
        } else if (new_state != PlannerState::TRACKING) {
            tracking_solve_fail_streak_ = 0;
            tracking_solve_success_streak_ = 0;
            tracking_feasibility_recovery_active_ = false;
        }
        if (new_state == PlannerState::SETTLING || state_ == PlannerState::SETTLING) {
            settling_step_count_ = 0;
        }
        if (new_state == PlannerState::SETTLING) {
            settling_enter_time_ = ros::Time::now();
        } else if (state_ == PlannerState::SETTLING) {
            settling_enter_time_ = ros::Time(0);
        }
        if (new_state == PlannerState::REACHED) {
            reached_time_ = ros::Time::now();
        } else if (state_ == PlannerState::REACHED) {
            reached_time_ = ros::Time(0);
        }
        ROS_INFO("[LocalPlannerROS] State: %s -> %s",
                 plannerStateToString(state_).c_str(),
                 plannerStateToString(new_state).c_str());
        state_ = new_state;
    }
}

void LocalPlannerROS::updateSloshEstimate() {
    if (!slosh_enabled_) {
        return;
    }

    if (has_prev_odom_ && !prev_odom_time_.isZero() && !current_odom_time_.isZero()) {
        // 用真实 odom 时间戳差分，避免假定固定控制周期
        double dt_odom = (current_odom_time_ - prev_odom_time_).toSec();
        if (dt_odom > 1e-4 && dt_odom < 1.0) {
            double ax_raw = (current_v_ - prev_v_) / dt_odom;
            double alpha_raw = (current_omega_ - prev_omega_) / dt_odom;
            double ay_raw = current_v_ * current_omega_;

            ax_filtered_ = accel_filter_alpha_ * ax_raw + (1.0 - accel_filter_alpha_) * ax_filtered_;
            ay_filtered_ = accel_filter_alpha_ * ay_raw + (1.0 - accel_filter_alpha_) * ay_filtered_;
            alpha_filtered_ = accel_filter_alpha_ * alpha_raw + (1.0 - accel_filter_alpha_) * alpha_filtered_;
        }
    }

    const bool use_imu_ay = use_imu_lateral_accel_ && has_imu_;
    const bool use_imu_omega = use_imu_yaw_rate_ && has_imu_;
    const bool use_imu_alpha = use_imu_alpha_z_ && has_imu_ && has_prev_imu_;

    if ((use_imu_lateral_accel_ || use_imu_yaw_rate_ || use_imu_alpha_z_) && !has_imu_) {
        ROS_WARN_THROTTLE(2.0, "[LocalPlannerROS] IMU input requested but no IMU message received on %s, fallback to odom-based slosh estimate",
                          imu_topic_.c_str());
    }

    ay_est_used_ = use_imu_ay ? imu_ay_filtered_ : ay_filtered_;
    const double omega_for_slosh = use_imu_omega ? imu_omega_z_filtered_ : current_omega_;
    omega_est_used_ = omega_for_slosh;
    alpha_est_used_ = use_imu_alpha ? imu_alpha_filtered_ : alpha_filtered_;

    // 当 offset_x/y=0 时，odom fallback 与 DiffDriveModel 的 slosh 输入映射一致；
    // 接入 IMU 后，优先使用实物测得的横向激励/角运动信息。
    slosh_integration_.update(ax_filtered_, ay_est_used_, omega_for_slosh, alpha_est_used_);

    prev_v_ = current_v_;
    prev_omega_ = current_omega_;
    prev_odom_time_ = current_odom_time_;
    has_prev_odom_ = true;
}

double LocalPlannerROS::computePredictedSloshHeightMax(const MPCSolution& solution) const {
    if (!slosh_enabled_ || solution.x_predicted.empty()) {
        return 0.0;
    }

    const double h_coeff = slosh_integration_.getModalParams().height_coeff;
    const double R = slosh_params_.container_radius;
    const double g = 9.81;

    double height_max = 0.0;
    const size_t n_states = solution.x_predicted.size();
    const size_t n_inputs = solution.u_optimal.size();

    for (size_t k = 0; k < n_states; ++k) {
        const StateVector& xk = solution.x_predicted[k];
        const double eta_x = xk(StateIndex::ETA_X);
        const double eta_y = xk(StateIndex::ETA_Y);
        const double eta_modal = h_coeff * std::hypot(eta_x, eta_y);

        double omega_k = 0.0;
        if (n_inputs > 0) {
            if (k < n_inputs) {
                omega_k = solution.u_optimal[k](ControlIndex::OMEGA);
            } else {
                omega_k = solution.u_optimal.back()(ControlIndex::OMEGA);
            }
        }

        double eta_parabola = 0.0;
        if (slosh_params_.use_parabola_term) {
            eta_parabola = (R * R * omega_k * omega_k) / (4.0 * g);
        }

        height_max = std::max(height_max, eta_modal + eta_parabola);
    }

    return height_max;
}

LocalPlannerROS::CostBreakdown LocalPlannerROS::computeCostBreakdown(
    const MPCSolution& solution,
    const std::vector<ReferencePoint>& refs,
    const MPCParams& params,
    const ControlVector& u_prev) const {

    CostBreakdown out;
    if (solution.x_predicted.empty()) {
        return out;
    }

    const int N = static_cast<int>(solution.u_optimal.size());
    const int n_states = static_cast<int>(solution.x_predicted.size());
    const int ramp_steps = std::max(1, params.terminal_ramp_steps);
    const int ramp_start = N - ramp_steps;

    auto terminal_factor = [&](int k, double configured_factor) {
        if (configured_factor <= 0.0 || k < ramp_start || k > N) {
            return 1.0;
        }
        const double alpha = static_cast<double>(k - ramp_start + 1)
                           / static_cast<double>(ramp_steps + 1);
        return 1.0 + alpha * (configured_factor - 1.0);
    };

    for (int k = 0; k < n_states; ++k) {
        const StateVector& x = solution.x_predicted[static_cast<size_t>(k)];
        const double e_l = x(StateIndex::E_L);
        const double e_c = x(StateIndex::E_C);
        const double e_theta = x(StateIndex::E_THETA);
        const double v = x(StateIndex::V);
        const double eta_x = x(StateIndex::ETA_X);
        const double eta_x_dot = x(StateIndex::ETA_X_DOT);
        const double eta_y = x(StateIndex::ETA_Y);
        const double eta_y_dot = x(StateIndex::ETA_Y_DOT);

        out.J_lag += (params.use_contour_lag ? params.Q_lag : params.Q_el) * e_l * e_l;
        out.J_contour += terminal_factor(k, params.terminal_factor_ec) *
                         (params.use_contour_lag ? params.Q_contour : params.Q_ec) * e_c * e_c;
        out.J_etheta += terminal_factor(k, params.terminal_factor_etheta) *
                        params.Q_etheta * e_theta * e_theta;

        const double v_factor = terminal_factor(k, params.terminal_factor_v);
        if (k < static_cast<int>(refs.size())) {
            const double dv = v - refs[static_cast<size_t>(k)].v_ref;
            out.J_v += v_factor * params.Q_v * dv * dv;
        } else {
            // buildQPCost() has no v_ref linear term at x_N; match the QP term.
            out.J_v += v_factor * params.Q_v * v * v;
        }

        if (params.Q_slosh_eta > 0.0) {
            const double preview_factor = (k > 0) ? std::max(0.0, params.slosh_preview_factor) : 0.0;
            out.J_slosh_eta +=
                (terminal_factor(k, params.terminal_factor_slosh_eta) + preview_factor) *
                params.Q_slosh_eta * (eta_x * eta_x + eta_y * eta_y);
        }
        if (params.Q_slosh_eta_dot > 0.0) {
            const double preview_factor = (k > 0) ? std::max(0.0, params.slosh_preview_factor) : 0.0;
            out.J_slosh_eta_dot +=
                (terminal_factor(k, params.terminal_factor_slosh_eta_dot) + preview_factor) *
                params.Q_slosh_eta_dot *
                (eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot);
        }
    }

    for (int k = 0; k < N; ++k) {
        const ControlVector& u = solution.u_optimal[static_cast<size_t>(k)];
        const double a = u(ControlIndex::A);
        const double omega = u(ControlIndex::OMEGA);

        out.J_control += params.R_a * a * a + params.R_omega * omega * omega;

        if (params.enable_omega_ff && k < static_cast<int>(refs.size())) {
            const double omega_ref = refs[static_cast<size_t>(k)].v_ref *
                                     refs[static_cast<size_t>(k)].kappa;
            const double domega_ref = omega - omega_ref;
            out.J_omega_ff += params.Q_omega_ff * domega_ref * domega_ref;
        }

        const ControlVector& up = (k == 0) ? u_prev : solution.u_optimal[static_cast<size_t>(k - 1)];
        const double da = a - up(ControlIndex::A);
        const double domega = omega - up(ControlIndex::OMEGA);
        out.J_smooth += params.R_da * da * da + params.R_domega * domega * domega;
    }

    out.J_total =
        out.J_lag + out.J_contour + out.J_etheta + out.J_v + out.J_omega_ff +
        out.J_control + out.J_smooth + out.J_slosh_eta + out.J_slosh_eta_dot;
    return out;
}

void LocalPlannerROS::publishCostBreakdown(const CostBreakdown& breakdown) {
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

void LocalPlannerROS::publishSloshHorizonSummary(const MPCSolution& solution) {
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

    if (!slosh_enabled_ || solution.x_predicted.empty()) {
        mpc_slosh_horizon_summary_pub_.publish(msg);
        return;
    }

    const double h_coeff = slosh_integration_.getModalParams().height_coeff;
    const double R = slosh_params_.container_radius;
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
        if (slosh_params_.use_parabola_term) {
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

void LocalPlannerROS::publishCmdVel(double v, double omega) {
    // 停车指令直接下发，同时重置滤波器状态
    if (std::abs(v) < 1e-6 && std::abs(omega) < 1e-6) {
        filtered_v_ = 0.0;
        filtered_omega_ = 0.0;
    } else {
        // 一阶指数移动平均（EMA）低通滤波
        filtered_v_ = cmd_filter_alpha_v_ * v
                     + (1.0 - cmd_filter_alpha_v_) * filtered_v_;
        // 曲率自适应 EMA：|omega| 大时自动提升 alpha（减弱滤波），避免急弯滞后
        double effective_alpha_omega = std::max(0.0, std::min(1.0,
            cmd_filter_alpha_omega_ + cmd_filter_kappa_boost_ * std::abs(omega)));
        filtered_omega_ = effective_alpha_omega * omega
                        + (1.0 - effective_alpha_omega) * filtered_omega_;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = filtered_v_;
    cmd.angular.z = filtered_omega_;
    cmd_vel_pub_.publish(cmd);
}

void LocalPlannerROS::publishSmoothedPath() {
    if (!path_params_.publish_smoothed_path || !smoothed_path_pub_) {
        return;
    }

    nav_msgs::Path path_out;
    if (!path_handler_.getSmoothedPath(path_out, path_params_.smoothed_path_points)) {
        return;
    }
    smoothed_path_pub_.publish(path_out);
}

void LocalPlannerROS::publishLocalPath(const std::vector<StateVector>& predicted_states,
                                       const std::vector<ReferencePoint>& refs) {
    if (local_path_pub_.getNumSubscribers() == 0) {
        return;
    }
    
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    const std::string out_frame = map_frame_.empty() ? base_frame_ : map_frame_;
    path.header.frame_id = out_frame;

    geometry_msgs::TransformStamped tf_base_to_out;
    bool use_tf = false;
    if (out_frame != base_frame_ && tf_buffer_) {
        try {
            tf_base_to_out = tf_buffer_->lookupTransform(
                out_frame, base_frame_, ros::Time(0), ros::Duration(0.05));
            use_tf = true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "[LocalPlannerROS] TF error in local_path: %s", ex.what());
            path.header.frame_id = base_frame_;
        }
    }
    
    // 预测的是 Frenet 误差：使用参考点恢复到笛卡尔坐标
    if (refs.empty()) {
        return;
    }

    // 可视化起点：当前 base_link 原点（确保 local_path 从车体开始）
    {
        geometry_msgs::PoseStamped pose_base;
        pose_base.header = path.header;
        pose_base.header.frame_id = base_frame_;
        pose_base.pose.position.x = 0.0;
        pose_base.pose.position.y = 0.0;
        pose_base.pose.position.z = 0.0;

        tf2::Quaternion q0;
        q0.setRPY(0.0, 0.0, 0.0);
        pose_base.pose.orientation = tf2::toMsg(q0);

        if (use_tf) {
            geometry_msgs::PoseStamped pose_out;
            tf2::doTransform(pose_base, pose_out, tf_base_to_out);
            pose_out.header.frame_id = out_frame;
            path.poses.push_back(pose_out);
        } else {
            pose_base.header.frame_id = path.header.frame_id;
            path.poses.push_back(pose_base);
        }
    }

    const size_t n_states = predicted_states.size();
    const size_t n_refs = refs.size();

    for (size_t i = 0; i < n_states; ++i) {
        const ReferencePoint& ref = refs[std::min(i, n_refs - 1)];
        const StateVector& x_state = predicted_states[i];

        // 获取 Frenet 误差
        const double e_l = x_state(StateIndex::E_L);
        const double e_c = x_state(StateIndex::E_C);
        const double e_theta = x_state(StateIndex::E_THETA);
        const double cos_t = std::cos(ref.theta_path);
        const double sin_t = std::sin(ref.theta_path);

        // 还原到笛卡尔坐标：只使用横向误差 e_c，忽略纵向误差 e_l
        // 这样在弯道上不会产生锯齿状效果
        // 纵向误差主要影响速度跟踪，对可视化位置影响较小
        const double px = ref.x - e_c * sin_t;
        const double py = ref.y + e_c * cos_t;
        const double theta = ref.theta_path + e_theta;

        geometry_msgs::PoseStamped pose_base;
        pose_base.header = path.header;
        pose_base.header.frame_id = base_frame_;
        pose_base.pose.position.x = px;
        pose_base.pose.position.y = py;
        pose_base.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta);
        pose_base.pose.orientation = tf2::toMsg(q);

        if (use_tf) {
            geometry_msgs::PoseStamped pose_out;
            tf2::doTransform(pose_base, pose_out, tf_base_to_out);
            pose_out.header.frame_id = out_frame;
            path.poses.push_back(pose_out);
        } else {
            pose_base.header.frame_id = path.header.frame_id;
            path.poses.push_back(pose_base);
        }
    }
    
    local_path_pub_.publish(path);
}

void LocalPlannerROS::publishReferencePath(const std::vector<ReferencePoint>& refs) {
    if (reference_path_pub_.getNumSubscribers() == 0 || refs.empty()) {
        return;
    }

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    const std::string out_frame = map_frame_.empty() ? base_frame_ : map_frame_;
    path.header.frame_id = out_frame;

    geometry_msgs::TransformStamped tf_base_to_out;
    bool use_tf = false;
    if (out_frame != base_frame_ && tf_buffer_) {
        try {
            tf_base_to_out = tf_buffer_->lookupTransform(
                out_frame, base_frame_, ros::Time(0), ros::Duration(0.05));
            use_tf = true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "[LocalPlannerROS] TF error in reference_path: %s", ex.what());
            path.header.frame_id = base_frame_;
        }
    }

    path.poses.reserve(refs.size());
    for (const auto& ref : refs) {
        geometry_msgs::PoseStamped pose_base;
        pose_base.header = path.header;
        pose_base.header.frame_id = base_frame_;
        pose_base.pose.position.x = ref.x;
        pose_base.pose.position.y = ref.y;
        pose_base.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, ref.theta_path);
        pose_base.pose.orientation = tf2::toMsg(q);

        if (use_tf) {
            geometry_msgs::PoseStamped pose_out;
            tf2::doTransform(pose_base, pose_out, tf_base_to_out);
            pose_out.header.frame_id = out_frame;
            path.poses.push_back(pose_out);
        } else {
            pose_base.header.frame_id = path.header.frame_id;
            path.poses.push_back(pose_base);
        }
    }

    reference_path_pub_.publish(path);
}

void LocalPlannerROS::publishStatus() {
    if (status_pub_.getNumSubscribers() == 0) {
        return;
    }
    
    std_msgs::String msg;
    msg.data = plannerStateToString(state_);
    status_pub_.publish(msg);
}

void LocalPlannerROS::publishTerminalDebug() {
    if (terminal_mode_pub_.getNumSubscribers() > 0) {
        std_msgs::String msg;
        msg.data = terminal_mode_debug_;
        terminal_mode_pub_.publish(msg);
    }

    if (terminal_recovery_latched_pub_.getNumSubscribers() > 0) {
        std_msgs::Int32 msg;
        msg.data = terminal_recovery_latched_ ? 1 : 0;
        terminal_recovery_latched_pub_.publish(msg);
    }

    if (terminal_v_envelope_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(last_terminal_v_envelope_);
        terminal_v_envelope_pub_.publish(msg);
    }

    if (terminal_envelope_active_pub_.getNumSubscribers() > 0) {
        std_msgs::Int32 msg;
        msg.data = last_terminal_envelope_active_;
        terminal_envelope_active_pub_.publish(msg);
    }

    if (terminal_phase_active_pub_.getNumSubscribers() > 0) {
        std_msgs::Int32 msg;
        msg.data = last_terminal_phase_active_;
        terminal_phase_active_pub_.publish(msg);
    }

    if (terminal_cmd_v_pre_clamp_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(last_terminal_cmd_v_pre_clamp_);
        terminal_cmd_v_pre_clamp_pub_.publish(msg);
    }

    if (terminal_cmd_v_post_clamp_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(last_terminal_cmd_v_post_clamp_);
        terminal_cmd_v_post_clamp_pub_.publish(msg);
    }

    if (profile_cap_active_pub_.getNumSubscribers() > 0) {
        std_msgs::Int32 msg;
        msg.data = last_profile_cap_active_;
        profile_cap_active_pub_.publish(msg);
    }

    auto publish_profile_float = [](ros::Publisher& pub, double value) {
        if (pub.getNumSubscribers() <= 0) {
            return;
        }
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(value);
        pub.publish(msg);
    };
    publish_profile_float(profile_cap_v_profile_pub_, last_profile_cap_v_profile_);
    publish_profile_float(profile_cap_cmd_v_pre_pub_, last_profile_cap_cmd_v_pre_);
    publish_profile_float(profile_cap_cmd_v_post_pub_, last_profile_cap_cmd_v_post_);
    publish_profile_float(profile_cap_implied_ax_pub_, last_profile_cap_implied_ax_);
    publish_profile_float(profile_cap_implied_jerk_pub_, last_profile_cap_implied_jerk_);

    if (terminal_goal_info_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32MultiArray msg;
        msg.data.resize(8, 0.0f);

        if (!terminal_goal_info_valid_) {
            const float nan = std::numeric_limits<float>::quiet_NaN();
            msg.data[0] = nan;
            msg.data[1] = nan;
            msg.data[2] = nan;
            msg.data[3] = nan;
            msg.data[4] = nan;
        } else {
            msg.data[0] = static_cast<float>(terminal_goal_info_debug_.dx);
            msg.data[1] = static_cast<float>(terminal_goal_info_debug_.dy);
            msg.data[2] = static_cast<float>(terminal_goal_info_debug_.dist);
            msg.data[3] = static_cast<float>(terminal_goal_info_debug_.bearing);
            msg.data[4] = static_cast<float>(terminal_goal_info_debug_.goal_yaw_err);
            msg.data[5] = terminal_goal_info_debug_.has_goal_yaw ? 1.0f : 0.0f;
            msg.data[6] = terminal_goal_info_debug_.position_reached ? 1.0f : 0.0f;
            msg.data[7] = terminal_goal_info_debug_.pose_reached ? 1.0f : 0.0f;
        }

        terminal_goal_info_pub_.publish(msg);
    }
}

void LocalPlannerROS::resetWarmStart(bool keep_u_prev, bool reset_slosh) {
    mpc_solver_.resetWarmStart(keep_u_prev);
    if (!keep_u_prev) {
        last_control_.setZero();
        filtered_v_ = 0.0;
        filtered_omega_ = 0.0;
    }
    // slosh 重置策略 (参考 MPC_INTEGRATION_NOTES §6.1)：
    // - keep_u_prev=false（ERROR/手动复位）：可选择完全重置 slosh + 滤波器
    // - keep_u_prev=true （路径跳变/重规划）：保留 slosh 物理连续性，只重置滤波器
    if (slosh_enabled_) {
        if (reset_slosh && !keep_u_prev) {
            slosh_integration_.reset();
        }
        // 加速度滤波器始终重置，避免旧差分值污染新路径段
        ax_filtered_ = 0.0;
        ay_filtered_ = 0.0;
        alpha_filtered_ = 0.0;
        has_prev_odom_ = false;
        prev_odom_time_ = ros::Time(0);
    }
    last_v_des_eff_ = 0.0;
    last_v_des_raw_ = 0.0;
    last_v_des_target_ = 0.0;
    last_v_des_rate_limited_active_ = 0;
    profile_cap_has_last_ax_ = false;
    profile_cap_last_ax_ = 0.0;
    last_profile_cap_active_ = 0;
}

void LocalPlannerROS::publishReferenceExecutionDebug(const std::vector<ReferencePoint>& refs) {
    if (refs.empty()) {
        return;
    }

    auto publish_float = [](ros::Publisher& pub, double value) {
        if (pub.getNumSubscribers() <= 0) {
            return;
        }
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(value);
        pub.publish(msg);
    };

    auto abs_p95 = [](std::vector<double> values) {
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
    };

    const double dt = std::max(1e-6, mpc_params_.dt);
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
            const double ax = (v_next - v) / dt;
            ax_values.push_back(ax);
            if (!has_first_ax) {
                first_ax = ax;
                has_first_ax = true;
            }
            if (has_prev_ax) {
                const double jerk = (ax - prev_ax) / dt;
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
    publish_float(ref_v_ref_pub_, ref0.v_ref);
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
    publish_float(ref_v_path_pub_, ref0.v_path);
    publish_float(ref_kappa_pub_, ref0.kappa);
    publish_float(ref_s_pub_, ref0.s);
    publish_float(ref_implied_ax_pub_, has_first_ax ? first_ax : 0.0);
    publish_float(ref_implied_ay_pub_, ay_values.empty() ? 0.0 : ay_values.front());
    publish_float(ref_implied_jerk_pub_, first_jerk);
    publish_float(ref_implied_ax_abs_p95_pub_, abs_p95(ax_values));
    publish_float(ref_implied_ay_abs_p95_pub_, abs_p95(ay_values));
    publish_float(ref_implied_jerk_abs_p95_pub_, abs_p95(jerk_values));
}

void LocalPlannerROS::publishSloshDebug(double solve_time_ms, bool solve_ok, bool publish_solver_debug) {
    if (slosh_episode_id_pub_.getNumSubscribers() > 0) {
        std_msgs::Int32 msg;
        msg.data = episode_id_;
        slosh_episode_id_pub_.publish(msg);
    }

    if (slosh_height_pred_max_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(last_predicted_height_max_);
        slosh_height_pred_max_pub_.publish(msg);
    }

    if (slosh_q_slosh_eta_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(slosh_enabled_ ? mpc_params_.Q_slosh_eta : 0.0);
        slosh_q_slosh_eta_pub_.publish(msg);
    }

    if (slosh_constraint_active_pub_.getNumSubscribers() > 0) {
        std_msgs::Int32 msg;
        msg.data = last_constraint_active_;
        slosh_constraint_active_pub_.publish(msg);
    }

    if (slosh_v_des_eff_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(last_v_des_eff_);
        slosh_v_des_eff_pub_.publish(msg);
    }

    if (ref_v_des_raw_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(last_v_des_raw_);
        ref_v_des_raw_pub_.publish(msg);
    }
    if (ref_v_des_target_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(last_v_des_target_);
        ref_v_des_target_pub_.publish(msg);
    }
    if (ref_v_des_eff_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(last_v_des_eff_);
        ref_v_des_eff_pub_.publish(msg);
    }
    if (ref_v_des_rate_limited_pub_.getNumSubscribers() > 0) {
        std_msgs::Int32 msg;
        msg.data = last_v_des_rate_limited_active_;
        ref_v_des_rate_limited_pub_.publish(msg);
    }

    if (slosh_omega_est_used_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(omega_est_used_);
        slosh_omega_est_used_pub_.publish(msg);
    }

    if (slosh_imu_omega_z_filtered_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(has_imu_ ? imu_omega_z_filtered_ : 0.0);
        slosh_imu_omega_z_filtered_pub_.publish(msg);
    }

    if (slosh_imu_ay_bias_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(imu_ay_bias_compensation_enable_ ? imu_ay_bias_ : 0.0);
        slosh_imu_ay_bias_pub_.publish(msg);
    }

    if (slosh_imu_ay_filtered_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(has_imu_ ? imu_ay_filtered_ : 0.0);
        slosh_imu_ay_filtered_pub_.publish(msg);
    }

    if (slosh_imu_ay_bias_ready_pub_.getNumSubscribers() > 0) {
        std_msgs::Int32 msg;
        msg.data = (imu_ay_bias_compensation_enable_ && imu_ay_bias_ready_) ? 1 : 0;
        slosh_imu_ay_bias_ready_pub_.publish(msg);
    }

    Eigen::Vector4d slosh_state = Eigen::Vector4d::Zero();
    if (slosh_enabled_) {
        slosh_state = slosh_integration_.getSloshState();
    }

    const double eta_norm =
        std::hypot(static_cast<double>(slosh_state(0)), static_cast<double>(slosh_state(2)));
    const double eta_dot_norm =
        std::hypot(static_cast<double>(slosh_state(1)), static_cast<double>(slosh_state(3)));
    const double omega0 = slosh_enabled_ ? slosh_integration_.getModalParams().omega_n : 0.0;
    const double modal_energy =
        omega0 * omega0 * eta_norm * eta_norm + eta_dot_norm * eta_dot_norm;
    const double modal_energy_norm = std::sqrt(std::max(0.0, modal_energy));

    // slosh 状态 [η_x, η̇_x, η_y, η̇_y]
    if (slosh_state_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32MultiArray msg;
        msg.data.resize(4);
        msg.data[0] = static_cast<float>(slosh_state(0));
        msg.data[1] = static_cast<float>(slosh_state(1));
        msg.data[2] = static_cast<float>(slosh_state(2));
        msg.data[3] = static_cast<float>(slosh_state(3));
        slosh_state_pub_.publish(msg);
    }

    if (slosh_eta_norm_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(eta_norm);
        slosh_eta_norm_pub_.publish(msg);
    }
    if (slosh_eta_dot_norm_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(eta_dot_norm);
        slosh_eta_dot_norm_pub_.publish(msg);
    }
    if (slosh_modal_energy_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(modal_energy);
        slosh_modal_energy_pub_.publish(msg);
    }
    if (slosh_modal_energy_norm_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(modal_energy_norm);
        slosh_modal_energy_norm_pub_.publish(msg);
    }
    if (slosh_excitation_ay_abs_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(std::abs(ay_est_used_));
        slosh_excitation_ay_abs_pub_.publish(msg);
    }
    if (slosh_excitation_alpha_abs_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(std::abs(alpha_est_used_));
        slosh_excitation_alpha_abs_pub_.publish(msg);
    }

    // 液面高度标量
    if (slosh_height_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = slosh_enabled_
                   ? static_cast<float>(slosh_integration_.getSloshHeight())
                   : 0.0f;
        slosh_height_pub_.publish(msg);
    }

    if (publish_solver_debug) {
        // 仅在本周期真正执行过 MPC solve 时发布 solver 调试，
        // 避免 REACHED/IDLE/ERROR 把上一帧结果重复刷到 bag 中。
        if (mpc_solve_ms_pub_.getNumSubscribers() > 0) {
            std_msgs::Float32 msg;
            msg.data = static_cast<float>(solve_time_ms);
            mpc_solve_ms_pub_.publish(msg);
        }

        if (mpc_status_val_pub_.getNumSubscribers() > 0) {
            std_msgs::Int32 msg;
            msg.data = solve_ok ? 1 : 0;
            mpc_status_val_pub_.publish(msg);
        }
    }

    // 加速度估计值（论文实验用）
    if (slosh_ax_est_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(ax_filtered_);
        slosh_ax_est_pub_.publish(msg);
    }
    if (slosh_ay_est_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(ay_est_used_);
        slosh_ay_est_pub_.publish(msg);
    }
    if (slosh_alpha_est_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(alpha_est_used_);
        slosh_alpha_est_pub_.publish(msg);
    }
}

}  // namespace scout_local_planner
