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
#include <deque>

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

            // 计算等效 QP 权重: Q_slosh_eta = Q_slosh * height_coeff²
            // 使得 J_slosh = Q_slosh * η² ≈ Q_slosh_eta * (xn² + yn²)
            double h_coeff = slosh_integration_.getModalParams().height_coeff;
            rs_h_coeff_ = h_coeff;
            mpc_params_.Q_slosh_eta = mpc_params_.Q_slosh * h_coeff * h_coeff;

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

            ROS_INFO("[LocalPlannerROS] Slosh integration enabled (Q_slosh=%.2f, h_coeff=%.4f, Q_slosh_eta=%.4f)",
                     mpc_params_.Q_slosh, h_coeff, mpc_params_.Q_slosh_eta);

            // 初始化风险调度器（需要 h_coeff）
            if (risk_scheduler_enable_) {
                pub_rho_k_    = nh_.advertise<std_msgs::Float32>("/risk_scheduler/rho_k", 1);
                pub_r_k_      = nh_.advertise<std_msgs::Float32>("/risk_scheduler/r_k", 1);
                pub_u_k_      = nh_.advertise<std_msgs::Float32>("/risk_scheduler/u_k", 1);
                pub_Q_eta_k_  = nh_.advertise<std_msgs::Float32>("/risk_scheduler/Q_eta_k", 1);
                pub_fallback_ = nh_.advertise<std_msgs::Bool>("/risk_scheduler/fallback_active", 1);
                ROS_INFO("[LocalPlannerROS] RiskScheduler enabled (h_coeff=%.4f)", rs_h_coeff_);
            }
        } else {
            slosh_enabled_ = false;
            risk_scheduler_enable_ = false;
            ROS_WARN("[LocalPlannerROS] DiffDriveModel cast failed, slosh disabled");
        }
    } else {
        slosh_enabled_ = false;
        risk_scheduler_enable_ = false;
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
    slosh_speed_governor_active_pub_ = nh_.advertise<std_msgs::Int32>("slosh/speed_governor_active", 1);
    slosh_omega_est_used_pub_ = nh_.advertise<std_msgs::Float32>("slosh/omega_est_used", 1);
    slosh_imu_omega_z_filtered_pub_ = nh_.advertise<std_msgs::Float32>("slosh/imu_omega_z_filtered", 1);
    slosh_imu_ay_bias_pub_ = nh_.advertise<std_msgs::Float32>("slosh/imu_ay_bias", 1);
    slosh_imu_ay_filtered_pub_ = nh_.advertise<std_msgs::Float32>("slosh/imu_ay_filtered", 1);
    slosh_imu_ay_bias_ready_pub_ = nh_.advertise<std_msgs::Int32>("slosh/imu_ay_bias_ready", 1);
    slosh_settling_time_pub_ = nh_.advertise<std_msgs::Float32>("slosh/settling_time", 1, true);
    mpc_solve_ms_pub_ = nh_.advertise<std_msgs::Float32>("mpc/solve_ms", 1);
    mpc_status_val_pub_ = nh_.advertise<std_msgs::Int32>("mpc/status_val", 1);
    terminal_mode_pub_ = nh_.advertise<std_msgs::String>("terminal/mode", 1);
    terminal_recovery_latched_pub_ = nh_.advertise<std_msgs::Int32>("terminal/recovery_latched", 1);
    terminal_goal_info_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("terminal/goal_info", 1);
    
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
    pnh.param("mpc/N", mpc_params_.N, 20);
    pnh.param("mpc/dt", mpc_params_.dt, 0.05);
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
    pnh.param("control_rate", control_rate_, 20.0);
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
              tracking_curvature_speed_cap_enable_, true);
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

    // 原地对齐模式
    pnh.param("heading_align/enable", heading_align_enable_, false);
    pnh.param("heading_align/enter_angle", heading_align_enter_, 0.8);
    pnh.param("heading_align/exit_angle", heading_align_exit_, 0.4);
    pnh.param("heading_align/omega_gain", heading_align_omega_gain_, 1.5);
    pnh.param("heading_align/max_omega", heading_align_max_omega_, 0.0);
    pnh.param("heading_align/start_distance", heading_align_start_dist_, 0.5);

    // 终点恢复（near-goal terminal recovery）
    pnh.param("terminal_recovery/enable", terminal_recovery_enable_, true);
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

    // ISR / ZV input shaping（baseline only）
    pnh.param("input_shaping/enable", input_shaping_enable_, false);
    pnh.param("input_shaping/type", input_shaping_type_, std::string("zv"));
    pnh.param("input_shaping/use_model_params", input_shaping_use_model_params_, true);
    pnh.param("input_shaping/omega0_override", input_shaping_omega0_override_, 0.0);
    pnh.param("input_shaping/zeta_override", input_shaping_zeta_override_, 0.0);

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

    // slosh-aware 速度治理
    pnh.param("slosh_speed_governor/enable", slosh_speed_governor_enable_, false);
    pnh.param("slosh_speed_governor/k_eta", slosh_k_eta_, 0.0);
    pnh.param("slosh_speed_governor/ay_max_base", slosh_ay_max_base_, 0.0);
    pnh.param("slosh_speed_governor/v_des_min", slosh_v_des_min_, 0.0);
    pnh.param("slosh_speed_governor/eta_deadband", slosh_eta_deadband_, 0.3);
    pnh.param("slosh_speed_governor/eta_exit_ratio", slosh_eta_exit_ratio_, 0.2);
    pnh.param("slosh_speed_governor/preview_distance", slosh_preview_distance_, 1.0);
    pnh.param("slosh_speed_governor/min_active_steps", slosh_min_active_steps_, 10);

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

    // ρ_k 风险自适应调度器参数
    pnh.param("risk_scheduler/enable", risk_scheduler_enable_, false);
    if (risk_scheduler_enable_) {
        RiskSchedulerParams rs_params;
        pnh.param("risk_scheduler/gamma",                 rs_params.gamma,                 5.0);
        pnh.param("risk_scheduler/rho_0",                 rs_params.rho_0,                 0.3);
        pnh.param("risk_scheduler/rate_limit_per_step",   rs_params.rate_limit_per_step,   0.05);
        pnh.param("risk_scheduler/Q_eta_min",             rs_params.Q_eta_min,             0.0);
        pnh.param("risk_scheduler/Q_eta_max",             rs_params.Q_eta_max,             10.0);
        pnh.param("risk_scheduler/eta_bar_max",           rs_params.eta_bar_max,           0.05);
        pnh.param("risk_scheduler/delta_eta_bar",         rs_params.delta_eta_bar,         0.02);
        pnh.param("risk_scheduler/beta",                  rs_params.beta,                  0.3);
        pnh.param("risk_scheduler/w_h",                   rs_params.w_h,                   0.4);
        pnh.param("risk_scheduler/w_e",                   rs_params.w_e,                   0.3);
        pnh.param("risk_scheduler/w_t",                   rs_params.w_t,                   0.3);
        pnh.param("risk_scheduler/w_r",                   rs_params.w_r,                   0.7);
        pnh.param("risk_scheduler/w_u",                   rs_params.w_u,                   0.3);
        pnh.param("risk_scheduler/u_threshold_high",      rs_params.u_threshold_high,      0.8);
        pnh.param("risk_scheduler/u_high_count_trigger",  rs_params.u_high_count_trigger,  10);
        pnh.param("risk_scheduler/imu_timeout_s",         rs_params.imu_timeout_s,         0.1);
        pnh.param("risk_scheduler/a_uncert_max",          rs_params.a_uncert_max,          0.5);
        pnh.param("risk_scheduler/Q_eta_fix",             rs_params.Q_eta_fix,             5.0);
        pnh.param("risk_scheduler/eta_bar_fix",           rs_params.eta_bar_fix,           0.04);
        pnh.param("risk_scheduler/d_goal_thresh",         rs_params.d_goal_thresh,         1.0);
        risk_scheduler_.init(rs_params);
    }
}

void LocalPlannerROS::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (path_handler_.updateGlobalPath(*msg, vehicle_params_.v_max * 0.8)) {
        has_path_ = true;
        resetInputShaper();
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
    imu_ay_unbiased_ = ay_raw - ay_bias;

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
        resetInputShaper();
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
                terminal_mode_debug_ = "GOAL_STOP_PENDING";
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

                if (!settling_active && terminal_recovery_enable_ && !goal_stop_pending_) {
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
                        publishTerminalDebug();

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

                if (goal_stop_pending_) {
                    // 终点最后一段继续交给 MPC 收敛，但将目标速度压到 0，
                    // 避免“进容差区后外层直接砍零”带来的冲过头与滑行。
                    slosh_governor_latched_ = false;
                    slosh_governor_hold_steps_ = 0;
                    last_speed_governor_active_ = 0;
                }

                // ── ρ_k 风险调度器 outer loop ──────────────────────────
                const double v_nominal = vehicle_params_.v_max * 0.8;
                MPCParams runtime_mpc_params = mpc_params_;
                if (!settling_active &&
                    risk_scheduler_enable_ &&
                    slosh_enabled_ &&
                    !goal_stop_pending_) {
                    const double d_goal_rs = path_handler_.getGoalDistance();
                    const ros::Time imu_stamp_rs = has_imu_ ? prev_imu_time_ : ros::Time(0);

                    risk_output_ = risk_scheduler_.update(
                        last_predicted_height_max_,   // 上一周期预测液面高度
                        E_slosh_prev_,                // 上一周期模态能量
                        std::isfinite(d_goal_rs) ? d_goal_rs : 1e6,
                        imu_ay_unbiased_,             // IMU 横向加速度（已去零偏）
                        current_v_ * current_omega_,  // 运动学离心估计
                        imu_stamp_rs,
                        imu_ay_bias_ready_,
                        v_nominal
                    );

                    // 将调度输出注入 MPC 参数（solve() 前完成）
                    runtime_mpc_params.Q_slosh_eta = risk_output_.Q_eta_k * rs_h_coeff_ * rs_h_coeff_;
                    if (runtime_mpc_params.enable_slosh_box_constraint && rs_h_coeff_ > 1e-9) {
                        const double denom = rs_h_coeff_ * std::sqrt(2.0);
                        runtime_mpc_params.slosh_eta_bar = risk_output_.eta_bar_k / denom;
                    }
                } else if (settling_active) {
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
                    runtime_mpc_params.Q_slosh_eta =
                        runtime_mpc_params.Q_slosh * rs_h_coeff_ * rs_h_coeff_;
                    if (slosh_enabled_ &&
                        runtime_mpc_params.enable_slosh_box_constraint &&
                        rs_h_coeff_ > 1e-9) {
                        const double denom = rs_h_coeff_ * std::sqrt(2.0);
                        runtime_mpc_params.slosh_eta_bar = settling_eta_bar_ / denom;
                    }
                }
                mpc_solver_.setMPCParams(runtime_mpc_params);

                double v_des_cmd_raw = (goal_stop_pending_ || settling_active) ? 0.0 :
                    (risk_scheduler_enable_ && slosh_enabled_) ?
                    risk_output_.v_ref_eff_k : v_nominal;
                double v_des_cmd = v_des_cmd_raw;
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
                            // omega_max 硬约束：v × kappa ≤ omega_max
                            // 该 cap 可低于 tracking_curvature_min_speed_，因路径几何确实要求低速
                            const double v_cap_omega =
                                (vehicle_params_.omega_max > 1e-3)
                                    ? vehicle_params_.omega_max / (kappa_preview_dbg + 1e-9)
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

                double v_des_target = v_des_cmd;
                last_speed_governor_active_ = 0;

                // 1. 获取参考点
                std::vector<ReferencePoint> ref_points;
                if (!path_handler_.getReferencePoints(
                        mpc_params_.N, mpc_params_.dt,
                        v_des_cmd,  // 执行层速度上限
                        v_nominal,  // 规划层名义速度，用于 v(s)
                        ref_points)) {
                    ROS_WARN_THROTTLE(1.0, "[LocalPlannerROS] Failed to get reference points");
                    publishCmdVel(0.0, 0.0);
                    return;
                }

                // 阶段 4：残余晃动感知的速度治理
                const double goal_dist = path_handler_.getGoalDistance();
                const bool near_goal_capture =
                    std::isfinite(goal_dist) &&
                    goal_dist > path_params_.goal_tolerance &&
                    goal_dist < path_params_.goal_capture_distance;

                if (near_goal_capture || !slosh_speed_governor_enable_ || !slosh_enabled_) {
                    slosh_governor_latched_ = false;
                    slosh_governor_hold_steps_ = 0;
                }

                if (!settling_active &&
                    slosh_speed_governor_enable_ &&
                    slosh_enabled_ &&
                    !ref_points.empty() &&
                    !near_goal_capture) {
                    const double slosh_height = slosh_integration_.getSloshHeight();
                    const double height_risk = std::max(slosh_height, last_predicted_height_max_);
                    const double height_limit = std::max(1e-6, mpc_params_.slosh_height_max);
                    const double eta_ratio = height_risk / height_limit;
                    const double eta_deadband = std::max(0.0, std::min(0.99, slosh_eta_deadband_));
                    const double eta_exit_ratio =
                        std::max(0.0, std::min(eta_deadband, slosh_eta_exit_ratio_));
                    const double ay_max_base = slosh_ay_max_base_ > 1e-6
                        ? slosh_ay_max_base_
                        : path_params_.max_lat_accel;

                    if (!slosh_governor_latched_) {
                        if (eta_ratio > eta_deadband) {
                            slosh_governor_latched_ = true;
                            slosh_governor_hold_steps_ = std::max(0, slosh_min_active_steps_);
                        }
                    } else {
                        if (slosh_governor_hold_steps_ > 0) {
                            --slosh_governor_hold_steps_;
                        }
                        if (slosh_governor_hold_steps_ <= 0 && eta_ratio < eta_exit_ratio) {
                            slosh_governor_latched_ = false;
                        }
                    }

                    if (ay_max_base > 1e-6 && slosh_governor_latched_) {
                        const double eta_excess = std::max(0.0, eta_ratio - eta_deadband);
                        const double scale = 1.0 / (1.0 + std::max(0.0, slosh_k_eta_) * eta_excess);
                        const double ay_budget_eff = ay_max_base * scale;
                        const double kappa_preview =
                            path_handler_.getMaxCurvatureAhead(path_params_.lookahead_distance,
                                                               slosh_preview_distance_);
                        if (kappa_preview > 1e-4) {
                            double v_cap = std::sqrt(std::max(0.0, ay_budget_eff / (kappa_preview + 1e-9)));
                            v_des_target = std::max(slosh_v_des_min_, std::min(v_des_cmd, v_cap));
                        }
                    }
                }

                // 对 v_des_eff 做变化率限制，避免每周期参考速度突跳引起求解震荡
                {
                    const double prev_v_des = last_v_des_eff_ > 1e-6 ? last_v_des_eff_ : v_des_cmd;
                    const double accel_limit = path_params_.max_tan_accel > 1e-6
                        ? path_params_.max_tan_accel
                        : std::max(1e-6, vehicle_params_.a_max);
                    const double decel_limit = path_params_.max_tan_decel > 1e-6
                        ? path_params_.max_tan_decel
                        : accel_limit;

                    const double v_lo = std::max(slosh_v_des_min_, prev_v_des - decel_limit * mpc_params_.dt);
                    const double v_hi = prev_v_des + accel_limit * mpc_params_.dt;
                    const double v_des_eff = std::max(v_lo, std::min(v_hi, v_des_target));
                    last_v_des_eff_ = std::max(slosh_v_des_min_, std::min(v_des_cmd, v_des_eff));

                    if (slosh_governor_latched_ && last_v_des_eff_ < v_des_cmd - 1e-3) {
                        last_speed_governor_active_ = 1;
                    }
                }

                if (last_speed_governor_active_) {
                    std::vector<ReferencePoint> ref_points_governed;
                    if (path_handler_.getReferencePoints(
                            mpc_params_.N, mpc_params_.dt,
                            last_v_des_eff_,
                            v_nominal,
                            ref_points_governed)) {
                        ref_points.swap(ref_points_governed);
                    } else {
                        last_speed_governor_active_ = 0;
                        last_v_des_eff_ = v_des_cmd;
                        slosh_governor_latched_ = false;
                        slosh_governor_hold_steps_ = 0;
                    }
                } else {
                    last_v_des_eff_ = v_des_cmd;
                }

                applyInputShaping(ref_points);

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
                mpc_solver_.setPreviousControl(last_control_);
                
                // 5. 求解 MPC
                MPCSolution solution = mpc_solver_.solve(current_state, ref_points);
                
                if (solution.success) {
                    // 6. 发布控制命令
                    // 注意：cmd_vel 是速度，不是加速度！
                    publishCmdVel(solution.v_cmd, solution.omega_cmd);
                    
                    // 保存控制量
                    last_control_ = solution.u_first;
                    
                    // 发布预测轨迹
                    publishLocalPath(solution.x_predicted, ref_points);
                    publishReferencePath(ref_points);

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
                    last_constraint_active_ =
                        (mpc_params_.slosh_height_max > 0.0 &&
                         last_predicted_height_max_ > mpc_params_.slosh_height_max) ? 1 : 0;
                    publishSloshDebug(solution.solve_time_ms, true);

                    // 保存本周期模态能量，供下周期风险调度器使用
                    if (risk_scheduler_enable_ && slosh_enabled_) {
                        const Eigen::Vector4d ss = slosh_integration_.getSloshState();
                        E_slosh_prev_ = ss(0) * ss(0) + ss(2) * ss(2);

                        // 发布风险调度器调试话题
                        {
                            std_msgs::Float32 msg;
                            msg.data = static_cast<float>(risk_output_.rho_k);
                            pub_rho_k_.publish(msg);
                            msg.data = static_cast<float>(risk_output_.r_k);
                            pub_r_k_.publish(msg);
                            msg.data = static_cast<float>(risk_output_.u_k);
                            pub_u_k_.publish(msg);
                            msg.data = static_cast<float>(risk_output_.Q_eta_k);
                            pub_Q_eta_k_.publish(msg);
                        }
                        {
                            std_msgs::Bool bmsg;
                            bmsg.data = risk_output_.fallback_active;
                            pub_fallback_.publish(bmsg);
                        }
                    }

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
                            (risk_scheduler_enable_ && slosh_enabled_ && risk_output_.fallback_active) ? 1 : 0,
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

    const bool goal_pose_reached = path_handler_.isGoalReached();
    const double goal_dist = path_handler_.getGoalDistance();
    const double goal_stop_release_dist =
        std::max(path_params_.goal_capture_distance, path_params_.goal_tolerance + 0.15);

    if (state_ == PlannerState::SETTLING) {
        terminal_recovery_latched_ = false;
        goal_stop_pending_ = false;

        const bool should_release =
            !goal_pose_reached &&
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
        const bool should_release =
            !std::isfinite(goal_dist) || goal_dist > goal_stop_release_dist;
        if (should_release) {
            goal_stop_pending_ = false;
        }
    }

    if (!goal_stop_pending_ && !goal_pose_reached) {
        return;
    }

    if (goal_pose_reached) {
        if (settling_enable_ && slosh_enabled_) {
            transitionTo(PlannerState::SETTLING);
            goal_stop_pending_ = false;
            terminal_recovery_latched_ = false;
            return;
        }
        goal_stop_pending_ = true;
    }

    const bool speed_low =
        std::abs(current_v_) < path_params_.goal_reached_max_speed &&
        std::abs(current_omega_) < path_params_.goal_reached_max_omega;
    if (speed_low) {
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
            resetInputShaper();
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
    last_speed_governor_active_ = 0;
    slosh_governor_latched_ = false;
    slosh_governor_hold_steps_ = 0;
    resetInputShaper();
}

void LocalPlannerROS::resetInputShaper() {
    input_shaping_v_history_.clear();
    input_shaping_A1_ = 1.0;
    input_shaping_A2_ = 0.0;
    input_shaping_delay_steps_ = 0;
    input_shaping_last_dt_ = 0.0;
}

void LocalPlannerROS::updateInputShaperConfig() {
    if (!input_shaping_enable_) {
        input_shaping_A1_ = 1.0;
        input_shaping_A2_ = 0.0;
        input_shaping_delay_steps_ = 0;
        return;
    }

    if (input_shaping_type_ != "zv") {
        ROS_WARN_THROTTLE(2.0, "[LocalPlannerROS] Unsupported input shaper type '%s', disabling shaping",
                          input_shaping_type_.c_str());
        input_shaping_A1_ = 1.0;
        input_shaping_A2_ = 0.0;
        input_shaping_delay_steps_ = 0;
        return;
    }

    double omega0 = 0.0;
    double zeta = 0.0;
    if (input_shaping_use_model_params_ && slosh_enabled_) {
        omega0 = slosh_integration_.getModalParams().omega_n;
        zeta = slosh_params_.damping_ratio;
    }
    if (input_shaping_omega0_override_ > 1e-6) {
        omega0 = input_shaping_omega0_override_;
    }
    if (input_shaping_zeta_override_ > 1e-6) {
        zeta = input_shaping_zeta_override_;
    }

    const double dt = std::max(1e-6, mpc_params_.dt);
    if (omega0 <= 1e-6) {
        input_shaping_A1_ = 1.0;
        input_shaping_A2_ = 0.0;
        input_shaping_delay_steps_ = 0;
        return;
    }

    zeta = std::max(0.0, std::min(0.99, zeta));
    const double wd = omega0 * std::sqrt(std::max(1e-9, 1.0 - zeta * zeta));
    if (wd <= 1e-6) {
        input_shaping_A1_ = 1.0;
        input_shaping_A2_ = 0.0;
        input_shaping_delay_steps_ = 0;
        return;
    }

    const double K = std::exp(-zeta * M_PI / std::sqrt(std::max(1e-9, 1.0 - zeta * zeta)));
    input_shaping_A1_ = 1.0 / (1.0 + K);
    input_shaping_A2_ = K / (1.0 + K);
    input_shaping_delay_steps_ = std::max(0, static_cast<int>(std::round((M_PI / wd) / dt)));

    if (std::abs(input_shaping_last_dt_ - dt) > 1e-9) {
        input_shaping_v_history_.clear();
        input_shaping_last_dt_ = dt;
    }
}

void LocalPlannerROS::applyInputShaping(std::vector<ReferencePoint>& refs) {
    if (!input_shaping_enable_ || refs.empty()) {
        return;
    }

    updateInputShaperConfig();
    if (input_shaping_delay_steps_ <= 0 || input_shaping_A2_ <= 1e-9) {
        return;
    }

    std::deque<double> hist = input_shaping_v_history_;
    std::vector<double> raw(refs.size(), 0.0);
    for (std::size_t i = 0; i < refs.size(); ++i) {
        raw[i] = std::max(0.0, refs[i].v_ref);
    }

    for (std::size_t i = 0; i < refs.size(); ++i) {
        double delayed = raw.front();
        if (static_cast<int>(i) >= input_shaping_delay_steps_) {
            delayed = raw[i - input_shaping_delay_steps_];
        } else {
            const std::size_t hist_need =
                static_cast<std::size_t>(input_shaping_delay_steps_ - static_cast<int>(i));
            if (hist_need <= hist.size()) {
                delayed = hist[hist.size() - hist_need];
            } else if (!hist.empty()) {
                delayed = hist.front();
            }
        }

        refs[i].v_ref = std::max(0.0, input_shaping_A1_ * raw[i] + input_shaping_A2_ * delayed);
    }

    for (double v : raw) {
        hist.push_back(v);
    }
    const std::size_t keep = static_cast<std::size_t>(std::max(1, input_shaping_delay_steps_));
    while (hist.size() > keep) {
        hist.pop_front();
    }
    input_shaping_v_history_.swap(hist);
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

    if (slosh_speed_governor_active_pub_.getNumSubscribers() > 0) {
        std_msgs::Int32 msg;
        msg.data = last_speed_governor_active_;
        slosh_speed_governor_active_pub_.publish(msg);
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

    // slosh 状态 [η_x, η̇_x, η_y, η̇_y]
    if (slosh_state_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32MultiArray msg;
        msg.data.resize(4);
        if (slosh_enabled_) {
            Eigen::Vector4d s = slosh_integration_.getSloshState();
            msg.data[0] = static_cast<float>(s(0));
            msg.data[1] = static_cast<float>(s(1));
            msg.data[2] = static_cast<float>(s(2));
            msg.data[3] = static_cast<float>(s(3));
        }
        slosh_state_pub_.publish(msg);
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
