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

void updateNormalizedSloshWeights(MPCParams& params, double h_coeff, double omega_n) {
    const double h_ref = std::max(1e-4, params.slosh_height_ref);
    params.Q_slosh_eta = params.Q_slosh * h_coeff * h_coeff / (h_ref * h_ref);

    if (params.slosh_eta_dot_ratio > 0.0) {
        params.Q_slosh_eta_dot = omega_n > 1e-6
            ? params.slosh_eta_dot_ratio * params.Q_slosh_eta / (omega_n * omega_n)
            : 0.0;
    }
}

double limitRate(double target, double current, double rate_limit, double dt) {
    if (!std::isfinite(target) || !std::isfinite(current) ||
        rate_limit <= 1e-6 || dt <= 1e-6) {
        return target;
    }
    const double max_delta = rate_limit * dt;
    return std::max(current - max_delta, std::min(current + max_delta, target));
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
    if (slosh_feedback_.imuRequired()) {
        const SloshFeedbackParams& sfp = slosh_feedback_.params();
        imu_sub_ = nh_.subscribe(slosh_feedback_.imuTopic(), 10,
                                 &LocalPlannerROS::imuCallback, this);
        ROS_INFO("[LocalPlannerROS] IMU interface enabled: topic=%s, ay=%s, omega_z=%s, alpha_z=%s",
                 sfp.imu_topic.c_str(),
                 sfp.use_imu_lateral_accel ? "on" : "off",
                 sfp.use_imu_yaw_rate ? "on" : "off",
                 sfp.use_imu_alpha_z ? "on" : "off");
        ROS_INFO("[LocalPlannerROS] IMU ay bias compensation: %s (init=%.2fs, |v|<%.3f, |omega|<%.3f, min_samples=%d)",
                 sfp.imu_ay_bias_compensation_enable ? "on" : "off",
                 sfp.imu_ay_bias_init_duration,
                 sfp.imu_ay_bias_static_v_max,
                 sfp.imu_ay_bias_static_omega_max,
                 sfp.imu_ay_bias_min_samples);
        ROS_INFO("[LocalPlannerROS] IMU ay bias estimator: first_static_only, ema_alpha=%.2f, trim_ratio=%.2f",
                 sfp.imu_ay_bias_estimator_alpha,
                 sfp.imu_ay_bias_trim_ratio);
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

    diagnostics_publisher_.advertise(nh_);
    
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

    SloshFeedbackParams slosh_feedback_params;
    pnh.param("slosh_estimator/accel_filter_alpha",
              slosh_feedback_params.accel_filter_alpha, 0.3);
    pnh.param("slosh_estimator/use_imu_lateral_accel",
              slosh_feedback_params.use_imu_lateral_accel, false);
    pnh.param("slosh_estimator/use_imu_yaw_rate",
              slosh_feedback_params.use_imu_yaw_rate, true);
    pnh.param("slosh_estimator/use_imu_alpha_z",
              slosh_feedback_params.use_imu_alpha_z, false);
    pnh.param("slosh_estimator/imu_topic",
              slosh_feedback_params.imu_topic, std::string("/imu/data"));
    pnh.param("slosh_estimator/imu_filter_alpha",
              slosh_feedback_params.imu_filter_alpha, 0.3);
    pnh.param("slosh_estimator/imu_ay_bias_compensation_enable",
              slosh_feedback_params.imu_ay_bias_compensation_enable, true);
    pnh.param("slosh_estimator/imu_ay_bias_init_duration",
              slosh_feedback_params.imu_ay_bias_init_duration, 3.0);
    pnh.param("slosh_estimator/imu_ay_bias_static_v_max",
              slosh_feedback_params.imu_ay_bias_static_v_max, 0.03);
    pnh.param("slosh_estimator/imu_ay_bias_static_omega_max",
              slosh_feedback_params.imu_ay_bias_static_omega_max, 0.03);
    pnh.param("slosh_estimator/imu_ay_bias_min_samples",
              slosh_feedback_params.imu_ay_bias_min_samples, 100);
    pnh.param("slosh_estimator/imu_ay_bias_estimator_alpha",
              slosh_feedback_params.imu_ay_bias_estimator_alpha, 0.15);
    pnh.param("slosh_estimator/imu_ay_bias_trim_ratio",
              slosh_feedback_params.imu_ay_bias_trim_ratio, 0.10);
    pnh.param("slosh_estimator/imu_ay_scale",
              slosh_feedback_params.imu_ay_scale, 1.0);
    slosh_feedback_.setParams(slosh_feedback_params);

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
    pnh.param("v_des_rate_limit/enable", v_des_rate_limit_enable_, true);
    pnh.param("v_des_rate_limit/accel_limit", v_des_accel_limit_, 0.6);
    pnh.param("v_des_rate_limit/decel_limit", v_des_decel_limit_, 0.8);
    ProfileExecutionCapParams profile_cap_params;
    pnh.param("external_profile_execution_cap/enable",
              profile_cap_params.enable, false);
    pnh.param("external_profile_execution_cap/accel_limit",
              profile_cap_params.accel_limit, 0.0);
    pnh.param("external_profile_execution_cap/decel_limit",
              profile_cap_params.decel_limit, 0.0);
    pnh.param("external_profile_execution_cap/jerk_limit",
              profile_cap_params.jerk_limit, 0.0);
    profile_execution_cap_.setParams(profile_cap_params);

    TerminalControllerParams terminal_params;
    pnh.param("terminal_capture_stop/goal_behind_x", terminal_params.goal_behind_x, -0.05);
    pnh.param("terminal_slowdown/enable", terminal_params.slowdown_enable, true);
    pnh.param("terminal_slowdown/distance", terminal_params.slowdown_distance, 1.20);
    pnh.param("terminal_slowdown/v_max", terminal_params.slowdown_v_max, 0.18);
    pnh.param("terminal_slowdown/Q_v", terminal_params.slowdown_q_v, 40.0);
    pnh.param("terminal_slowdown/terminal_factor_v",
              terminal_params.slowdown_terminal_factor_v, 5.0);
    pnh.param("terminal_capture_stop/enable", terminal_params.capture_stop_enable, true);
    pnh.param("terminal_capture_stop/distance", terminal_params.capture_stop_distance, 0.70);
    pnh.param("terminal_capture_stop/v_cap", terminal_params.capture_v_cap, 0.18);
    terminal_controller_.setParams(terminal_params);

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
    slosh_feedback_.onOdom(current_v_, current_omega_, current_odom_time_);
    
    // 更新位姿（从 odom 消息中提取）
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    
    // 更新 PathHandler 的机器人状态（关键！）
    path_handler_.updateRobotState(current_pose_, current_v_, current_omega_);
}

void LocalPlannerROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    slosh_feedback_.onImu(
        msg->linear_acceleration.y,
        msg->angular_velocity.z,
        stamp,
        current_v_,
        current_omega_,
        has_odom_);
}

void LocalPlannerROS::controlLoop(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 更新状态
    updateState();

    // 路径跳变/重规划提示：重置 warm-start
    if (path_handler_.consumeResetHint()) {
        resetWarmStart(true);
        tracking_solve_fail_streak_ = 0;
        tracking_solve_success_streak_ = 0;
        tracking_feasibility_recovery_active_ = false;
        tracking_reentry_ramp_steps_left_ =
            std::max(0, tracking_reentry_ramp_steps_);
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
            terminal_mode_debug_ = "SETTLING_DISABLED";
            break;
        case PlannerState::REACHED:
            terminal_mode_debug_ = "REACHED";
            break;
        case PlannerState::TRACKING:
        default:
            terminal_mode_debug_ = terminal_controller_.modeDebug();
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
            
        case PlannerState::TRACKING:
            // 执行 MPC 控制
            {
                GoalInfo goal_info;
                const bool has_goal_info = path_handler_.getGoalInfo(goal_info);

                const double v_nominal = vehicle_params_.v_max * 0.8;
                MPCParams runtime_mpc_params = mpc_params_;
                const double goal_dist_now = path_handler_.getGoalDistance();
                double a_brake =
                    path_params_.max_tan_decel > 1e-6 ? path_params_.max_tan_decel
                                                       : vehicle_params_.a_max;
                if (v_des_rate_limit_enable_ && v_des_decel_limit_ > 1e-6) {
                    a_brake = std::min(a_brake, v_des_decel_limit_);
                }
                const TerminalPlan terminal_plan =
                    terminal_controller_.plan(
                        has_goal_info,
                        goal_info,
                        goal_dist_now,
                        v_nominal,
                        path_params_,
                        a_brake);
                const bool in_terminal_phase = terminal_plan.terminal_phase;
                terminal_mode_debug_ = terminal_controller_.modeDebug();
                last_terminal_v_envelope_ = terminal_plan.v_envelope;
                last_terminal_envelope_active_ = terminal_plan.envelope_active ? 1 : 0;
                last_terminal_phase_active_ = terminal_plan.terminal_phase ? 1 : 0;

                if (in_terminal_phase) {
                    runtime_mpc_params.Q_v =
                        std::max(runtime_mpc_params.Q_v,
                                 terminal_controller_.params().slowdown_q_v);
                    runtime_mpc_params.terminal_factor_v =
                        std::max(runtime_mpc_params.terminal_factor_v,
                                 terminal_controller_.params().slowdown_terminal_factor_v);
                }

                const double v_des_cmd_raw = terminal_plan.v_des_raw;
                double v_des_cmd = std::min(v_des_cmd_raw, terminal_plan.v_envelope);

                mpc_solver_.setMPCParams(runtime_mpc_params);

                int reentry_steps_dbg = tracking_reentry_ramp_steps_left_;
                int tracking_fail_streak_dbg = tracking_solve_fail_streak_;
                int tracking_feas_active_dbg = tracking_feasibility_recovery_active_ ? 1 : 0;
                if (state_ == PlannerState::TRACKING &&
                    !terminal_controller_.goalStopPending() &&
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

                const double v_des_cmd_capped = v_des_cmd;
                double v_des_target = v_des_cmd_capped;

                // 对所有执行层 v_des 做变化率限制，避免速度参考突跳制造纵向 ax 脉冲。
                {
                    last_v_des_raw_ = v_des_cmd_raw;
                    last_v_des_target_ = v_des_target;
                    last_v_des_rate_limited_active_ = 0;

                    double v_des_eff = v_des_target;
                    if (v_des_rate_limit_enable_) {
                        const bool terminal_stop_target =
                            terminal_controller_.goalStopPending() || v_des_target <= 1e-6;
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
                        terminal_controller_.goalStopPending()
                            ? std::max(0.0, current_v_)
                            : v_des_cmd_capped;
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
                    const DiagnosticsCostBreakdown cost_breakdown =
                        computeCostBreakdown(solution, ref_points, runtime_mpc_params, u_prev_for_cost);

                    // 6. 发布控制命令
                    // 注意：cmd_vel 是速度，不是加速度！
                    // Terminal phase 用 envelope 兜底: MPC 输出超过运动学停车包络时硬 clamp,
                    // 越过 goal (dx ≤ 0) 时强制 0。omega 不做 envelope (不是 overshoot 主因)。
                    const double dt_cmd =
                        control_rate_ > 1e-3 ? 1.0 / control_rate_ : mpc_params_.dt;
                    const TerminalClampOutput terminal_clamp =
                        terminal_controller_.clampCommand(
                            solution.v_cmd,
                            filtered_v_,
                            dt_cmd,
                            has_goal_info,
                            goal_info,
                            terminal_plan,
                            a_brake);
                    double cmd_v_out = terminal_clamp.cmd_v;
                    last_terminal_cmd_v_pre_clamp_ = terminal_clamp.cmd_v_pre;
                    last_terminal_cmd_v_post_clamp_ = terminal_clamp.cmd_v_post;
                    last_profile_cap_active_ = 0;
                    last_profile_cap_v_profile_ = std::numeric_limits<double>::quiet_NaN();
                    last_profile_cap_cmd_v_pre_ = cmd_v_out;
                    last_profile_cap_cmd_v_post_ = cmd_v_out;
                    last_profile_cap_implied_ax_ = std::numeric_limits<double>::quiet_NaN();
                    last_profile_cap_implied_jerk_ = std::numeric_limits<double>::quiet_NaN();
                    const ProfileExecutionCapOutput profile_out =
                        profile_execution_cap_.apply(
                            cmd_v_out,
                            filtered_v_,
                            dt_cmd,
                            path_handler_,
                            path_params_,
                            vehicle_params_);
                    cmd_v_out = profile_out.cmd_v;
                    last_profile_cap_active_ = profile_out.active;
                    last_profile_cap_v_profile_ = profile_out.v_profile;
                    last_profile_cap_cmd_v_pre_ = profile_out.cmd_v_pre;
                    last_profile_cap_cmd_v_post_ = profile_out.cmd_v_post;
                    last_profile_cap_implied_ax_ = profile_out.implied_ax;
                    last_profile_cap_implied_jerk_ = profile_out.implied_jerk;
                    if (in_terminal_phase || profile_out.applied) {
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
                            "Qv=%.2f Qeta=%.2f "
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
        if (state_ == PlannerState::TRACKING) {
            transitionTo(PlannerState::ERROR);
            ROS_WARN("[LocalPlannerROS] Path invalid or timeout");
        }
        return;
    }
    
    if (state_ != PlannerState::TRACKING) {
        terminal_controller_.clearPending();
        return;
    }

    GoalInfo goal_info;
    const bool has_goal_info = path_handler_.getGoalInfo(goal_info);
    const double goal_dist = path_handler_.getGoalDistance();
    const TerminalStateUpdate terminal_update =
        terminal_controller_.updateState(
            has_goal_info,
            goal_info,
            goal_dist,
            current_v_,
            current_omega_,
            path_params_);
    terminal_mode_debug_ = terminal_controller_.modeDebug();
    if (terminal_update.reached) {
        transitionTo(PlannerState::REACHED);
        // 到达终点后保留 slosh 内部状态一段时间，便于观测残余晃动衰减。
        resetWarmStart(false, false);
    }
}

void LocalPlannerROS::transitionTo(PlannerState new_state) {
    if (state_ != new_state) {
        if (new_state == PlannerState::TRACKING && state_ != PlannerState::TRACKING) {
            ++episode_id_;
            tracking_reentry_ramp_steps_left_ =
                std::max(0, tracking_reentry_ramp_steps_);
            tracking_solve_fail_streak_ = 0;
            tracking_solve_success_streak_ = 0;
            tracking_feasibility_recovery_active_ = false;
        } else if (new_state != PlannerState::TRACKING) {
            tracking_solve_fail_streak_ = 0;
            tracking_solve_success_streak_ = 0;
            tracking_feasibility_recovery_active_ = false;
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

    slosh_feedback_output_ =
        slosh_feedback_.update(current_v_, current_omega_, current_odom_time_);

    // 当 offset_x/y=0 时，odom fallback 与 DiffDriveModel 的 slosh 输入映射一致；
    // 接入 IMU 后，优先使用实物测得的横向激励/角运动信息。
    slosh_integration_.update(
        slosh_feedback_output_.ax,
        slosh_feedback_output_.ay,
        slosh_feedback_output_.omega,
        slosh_feedback_output_.alpha);
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

DiagnosticsCostBreakdown LocalPlannerROS::computeCostBreakdown(
    const MPCSolution& solution,
    const std::vector<ReferencePoint>& refs,
    const MPCParams& params,
    const ControlVector& u_prev) const {

    DiagnosticsCostBreakdown out;
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

void LocalPlannerROS::publishCostBreakdown(const DiagnosticsCostBreakdown& breakdown) {
    diagnostics_publisher_.publishCostBreakdown(breakdown);
}

void LocalPlannerROS::publishSloshHorizonSummary(const MPCSolution& solution) {
    const double h_coeff =
        slosh_enabled_ ? slosh_integration_.getModalParams().height_coeff : 0.0;
    diagnostics_publisher_.publishSloshHorizonSummary(
        solution, slosh_enabled_, h_coeff, slosh_params_);
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
    TerminalDebugData data;
    data.mode = terminal_mode_debug_;
    data.goal_info = terminal_goal_info_debug_;
    data.goal_info_valid = terminal_goal_info_valid_;
    data.v_envelope = last_terminal_v_envelope_;
    data.envelope_active = last_terminal_envelope_active_;
    data.phase_active = last_terminal_phase_active_;
    data.cmd_v_pre_clamp = last_terminal_cmd_v_pre_clamp_;
    data.cmd_v_post_clamp = last_terminal_cmd_v_post_clamp_;
    data.profile_cap_active = last_profile_cap_active_;
    data.profile_cap_v_profile = last_profile_cap_v_profile_;
    data.profile_cap_cmd_v_pre = last_profile_cap_cmd_v_pre_;
    data.profile_cap_cmd_v_post = last_profile_cap_cmd_v_post_;
    data.profile_cap_implied_ax = last_profile_cap_implied_ax_;
    data.profile_cap_implied_jerk = last_profile_cap_implied_jerk_;
    diagnostics_publisher_.publishTerminalDebug(data);
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
        slosh_feedback_.resetOdomFilters();
        slosh_feedback_output_ = slosh_feedback_.output();
    }
    last_v_des_eff_ = 0.0;
    last_v_des_raw_ = 0.0;
    last_v_des_target_ = 0.0;
    last_v_des_rate_limited_active_ = 0;
    profile_execution_cap_.reset();
    last_profile_cap_active_ = 0;
}

void LocalPlannerROS::publishReferenceExecutionDebug(const std::vector<ReferencePoint>& refs) {
    diagnostics_publisher_.publishReferenceExecutionDebug(refs, mpc_params_.dt);
}

void LocalPlannerROS::publishSloshDebug(double solve_time_ms, bool solve_ok, bool publish_solver_debug) {
    SloshDebugData data;
    data.episode_id = episode_id_;
    data.predicted_height_max = last_predicted_height_max_;
    data.q_slosh_eta = slosh_enabled_ ? mpc_params_.Q_slosh_eta : 0.0;
    data.constraint_active = last_constraint_active_;
    data.v_des_eff = last_v_des_eff_;
    data.v_des_raw = last_v_des_raw_;
    data.v_des_target = last_v_des_target_;
    data.v_des_rate_limited_active = last_v_des_rate_limited_active_;
    data.feedback = slosh_feedback_output_;
    data.imu_ay_bias_compensation_enable =
        slosh_feedback_.params().imu_ay_bias_compensation_enable;
    data.slosh_enabled = slosh_enabled_;
    if (slosh_enabled_) {
        data.slosh_state = slosh_integration_.getSloshState();
        data.omega_n = slosh_integration_.getModalParams().omega_n;
        data.slosh_height = slosh_integration_.getSloshHeight();
    }
    data.solve_time_ms = solve_time_ms;
    data.solve_ok = solve_ok;
    data.publish_solver_debug = publish_solver_debug;
    diagnostics_publisher_.publishSloshDebug(data);
}

}  // namespace scout_local_planner
