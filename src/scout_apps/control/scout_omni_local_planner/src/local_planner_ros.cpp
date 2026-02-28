/**
 * @file local_planner_ros.cpp
 * @brief 全向轮 MPC 局部规划器 ROS 接口实现
 * 
 * 与 scout_local_planner 的主要区别：
 *   1. odom 读取 linear.y → current_vy_
 *   2. 状态向量含 V_X, V_Y（9 维）
 *   3. publishCmdVel(vx, vy, omega) 设置 linear.y
 *   4. 参数: Q_vx/Q_vy, R_ax/R_ay, vx_max/vy_max 等
 */

#include "scout_omni_local_planner/local_planner_ros.h"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace scout_omni_local_planner {

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
        ROS_ERROR("[OmniLocalPlannerROS] Failed to initialize MPC solver");
        return false;
    }
    
    // 订阅者
    global_path_sub_ = nh_.subscribe("global_path", 1, 
                                      &LocalPlannerROS::globalPathCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, 
                               &LocalPlannerROS::odomCallback, this);
    
    // 发布者
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("local_path", 1);
    if (path_params_.publish_smoothed_path) {
        smoothed_path_pub_ = nh_.advertise<nav_msgs::Path>(
            path_params_.smoothed_path_topic, 1);
    }
    status_pub_ = nh_.advertise<std_msgs::String>("mpc_status", 1);
    
    // 控制定时器
    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / control_rate_),
        &LocalPlannerROS::controlLoop, this);
    
    ROS_INFO("[OmniLocalPlannerROS] Initialized successfully (omni 3-DOF)");
    ROS_INFO("  - Control rate: %.1f Hz", control_rate_);
    ROS_INFO("  - MPC horizon: N=%d, dt=%.3f", mpc_params_.N, mpc_params_.dt);
    ROS_INFO("  - State: 9D [e_l, e_c, e_θ, v_x, v_y, η_x, η̇_x, η_y, η̇_y]");
    ROS_INFO("  - Control: 3D [a_x, a_y, ω]");
    
    return true;
}

void LocalPlannerROS::run() {
    ros::spin();
}

void LocalPlannerROS::loadParameters(ros::NodeHandle& pnh) {
    // MPC 参数
    pnh.param("mpc/N", mpc_params_.N, 20);
    pnh.param("mpc/dt", mpc_params_.dt, 0.05);
    pnh.param("mpc/Q_el", mpc_params_.Q_el, 1.0);
    pnh.param("mpc/Q_ec", mpc_params_.Q_ec, 10.0);
    pnh.param("mpc/Q_etheta", mpc_params_.Q_etheta, 5.0);
    pnh.param("mpc/Q_vx", mpc_params_.Q_vx, 1.0);      // 全向: Q_vx (替代 Q_v)
    pnh.param("mpc/Q_vy", mpc_params_.Q_vy, 5.0);       // 全向新增: 横向速度权重
    pnh.param("mpc/use_contour_lag", mpc_params_.use_contour_lag, false);
    pnh.param("mpc/Q_contour", mpc_params_.Q_contour, mpc_params_.Q_ec);
    pnh.param("mpc/Q_lag", mpc_params_.Q_lag, mpc_params_.Q_el);
    pnh.param("mpc/enable_omega_ff", mpc_params_.enable_omega_ff, false);
    pnh.param("mpc/Q_omega_ff", mpc_params_.Q_omega_ff, 0.0);
    pnh.param("mpc/terminal_factor_ec", mpc_params_.terminal_factor_ec, 1.0);
    pnh.param("mpc/terminal_factor_etheta", mpc_params_.terminal_factor_etheta, 1.0);
    pnh.param("mpc/terminal_factor_vx", mpc_params_.terminal_factor_vx, 1.0);  // 全向: terminal_factor_vx
    pnh.param("mpc/R_ax", mpc_params_.R_ax, 1.0);       // 全向: R_ax (替代 R_a)
    pnh.param("mpc/R_ay", mpc_params_.R_ay, 1.0);       // 全向新增: 横向加速度权重
    pnh.param("mpc/R_omega", mpc_params_.R_omega, 0.1);
    pnh.param("mpc/R_dax", mpc_params_.R_dax, 0.1);     // 全向: R_dax (替代 R_da)
    pnh.param("mpc/R_day", mpc_params_.R_day, 0.1);     // 全向新增: 横向加速度变化权重
    pnh.param("mpc/R_domega", mpc_params_.R_domega, 0.1);
    pnh.param("mpc/constrain_omega_rate", mpc_params_.constrain_omega_rate, true);
    pnh.param("mpc/constrain_accel_rate", mpc_params_.constrain_accel_rate, false);
    pnh.param("mpc/Q_slosh", mpc_params_.Q_slosh, 0.0);
    
    // 车辆参数（全向轮）
    pnh.param("vehicle/vx_max", vehicle_params_.vx_max, 1.0);
    pnh.param("vehicle/vx_min", vehicle_params_.vx_min, -0.3);
    pnh.param("vehicle/vy_max", vehicle_params_.vy_max, 0.5);    // 全向新增
    pnh.param("vehicle/omega_max", vehicle_params_.omega_max, 1.0);
    pnh.param("vehicle/ax_max", vehicle_params_.ax_max, 0.5);    // 全向: ax_max
    pnh.param("vehicle/ay_max", vehicle_params_.ay_max, 0.5);    // 全向新增
    pnh.param("vehicle/alpha_max", vehicle_params_.alpha_max, 1.0);
    pnh.param("vehicle/jx_max", vehicle_params_.jx_max, 0.0);    // 全向: jx_max
    pnh.param("vehicle/track_width", vehicle_params_.track_width, 0.456);
    pnh.param("vehicle/wheelbase", vehicle_params_.wheelbase, 0.451);
    pnh.param("vehicle/wheel_radius", vehicle_params_.wheel_radius, 0.09);
    
    // 路径处理参数
    pnh.param("path_handler/lookahead_distance", path_params_.lookahead_distance, 1.0);
    pnh.param("path_handler/goal_tolerance", path_params_.goal_tolerance, 0.1);
    pnh.param("path_handler/yaw_tolerance", path_params_.yaw_tolerance, 0.1);
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
    pnh.param("path_handler/publish_smoothed_path",
              path_params_.publish_smoothed_path, false);
    pnh.param("path_handler/smoothed_path_topic",
              path_params_.smoothed_path_topic,
              std::string("global_path_smooth"));
    pnh.param("path_handler/smoothed_path_points",
              path_params_.smoothed_path_points, 80);
    pnh.param("path_handler/min_path_points",
              path_params_.min_path_points, 2);
    
    // 其他参数
    pnh.param("control_rate", control_rate_, 20.0);
    pnh.param("base_frame", base_frame_, std::string("base_link"));
    pnh.param("map_frame", map_frame_, std::string("map"));
    pnh.param("verbose", verbose_, false);
    pnh.param("safety/infeasible_decel", infeasible_decel_, 1.0);
    pnh.param("safety/infeasible_omega_scale", infeasible_omega_scale_, 0.0);
    pnh.param("safety/infeasible_min_speed", infeasible_min_speed_, 0.0);

    // 原地对齐模式
    pnh.param("heading_align/enable", heading_align_enable_, false);
    pnh.param("heading_align/enter_angle", heading_align_enter_, 0.8);
    pnh.param("heading_align/exit_angle", heading_align_exit_, 0.4);
    pnh.param("heading_align/omega_gain", heading_align_omega_gain_, 1.5);
    pnh.param("heading_align/max_omega", heading_align_max_omega_, 0.0);
    pnh.param("heading_align/start_distance", heading_align_start_dist_, 0.5);
    
    // 将 base_frame 传递给 path_handler
    path_params_.base_frame = base_frame_;
}

void LocalPlannerROS::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (path_handler_.updateGlobalPath(*msg, vehicle_params_.vx_max * 0.8)) {
        has_path_ = true;
        resetWarmStart(true);
        
        if (state_ == PlannerState::IDLE || 
            state_ == PlannerState::REACHED ||
            state_ == PlannerState::ERROR) {
            transitionTo(PlannerState::TRACKING);
        }
    }
}

void LocalPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    current_vx_ = msg->twist.twist.linear.x;
    current_vy_ = msg->twist.twist.linear.y;   // 全向轮：读取横向速度
    current_omega_ = msg->twist.twist.angular.z;
    has_odom_ = true;
    
    // 更新位姿（从 odom 消息中提取）
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    
    // 更新 PathHandler 的机器人状态
    // PathHandler 接受的 v 是纵向速度（用于弧长推进）
    path_handler_.updateRobotState(current_pose_, current_vx_, current_omega_);
}

void LocalPlannerROS::controlLoop(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 更新状态
    updateState();

    // 路径跳变/重规划提示：重置 warm-start
    if (path_handler_.consumeResetHint()) {
        resetWarmStart(true);
    }

    if (state_ != PlannerState::TRACKING) {
        heading_align_active_ = false;
    }
    
    // 发布状态
    publishStatus();
    
    // 根据状态机执行
    switch (state_) {
        case PlannerState::IDLE:
        case PlannerState::ERROR:
            // 停止（全向轮：三个速度都归零）
            publishCmdVel(0.0, 0.0, 0.0);
            break;
            
        case PlannerState::REACHED:
            // 到达目标，停止
            publishCmdVel(0.0, 0.0, 0.0);
            ROS_INFO_THROTTLE(5.0, "[OmniLocalPlannerROS] Goal reached");
            break;
            
        case PlannerState::TRACKING:
            // 执行 MPC 控制
            {
                // 1. 获取参考点
                std::vector<ReferencePoint> ref_points;
                if (!path_handler_.getReferencePoints(
                        mpc_params_.N, mpc_params_.dt, 
                        vehicle_params_.vx_max * 0.8,  // 期望纵向速度
                        ref_points)) {
                    ROS_WARN_THROTTLE(1.0, "[OmniLocalPlannerROS] Failed to get reference points");
                    publishCmdVel(0.0, 0.0, 0.0);
                    return;
                }

                publishSmoothedPath();
                
                // 2. 获取 Frenet 误差
                FrenetState frenet;
                if (!path_handler_.getFrenetState(frenet)) {
                    ROS_WARN_THROTTLE(1.0, "[OmniLocalPlannerROS] Failed to get Frenet state");
                    publishCmdVel(0.0, 0.0, 0.0);
                    return;
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
                    double omega = -heading_align_omega_gain_ * frenet.e_theta;
                    omega = std::max(-max_omega, std::min(max_omega, omega));
                    // 全向轮原地对齐：vx=0, vy=0, 只转
                    publishCmdVel(0.0, 0.0, omega);

                    if (verbose_) {
                        ROS_INFO_THROTTLE(0.5, "[Align] e_theta=%.3f, omega=%.3f", 
                                          frenet.e_theta, omega);
                    }
                    return;
                }
                
                // 3. 构建当前状态（9 维全向增广）
                auto clamp = [](double v, double lo, double hi) {
                    return std::max(lo, std::min(hi, v));
                };

                double vx_clamped = clamp(current_vx_, vehicle_params_.vx_min, vehicle_params_.vx_max);
                double vy_clamped = clamp(current_vy_, -vehicle_params_.vy_max, vehicle_params_.vy_max);

                StateVector current_state;
                current_state.setZero();  // 初始化所有状态（包括晃动状态）
                current_state(StateIndex::E_L) = frenet.e_l;
                current_state(StateIndex::E_C) = frenet.e_c;
                current_state(StateIndex::E_THETA) = frenet.e_theta;
                current_state(StateIndex::V_X) = vx_clamped;
                current_state(StateIndex::V_Y) = vy_clamped;   // 全向新增
                
                // 4. 设置上一步控制量
                mpc_solver_.setPreviousControl(last_control_);
                
                // 5. 求解 MPC
                MPCSolution solution = mpc_solver_.solve(current_state, ref_points);
                
                if (solution.success) {
                    // 6. 发布控制命令（全向轮：三自由度）
                    publishCmdVel(solution.vx_cmd, solution.vy_cmd, solution.omega_cmd);
                    
                    // 保存控制量
                    last_control_ = solution.u_first;
                    
                    // 发布预测轨迹
                    publishLocalPath(solution.x_predicted, ref_points);
                    
                    if (verbose_) {
                        ROS_INFO_THROTTLE(0.5, 
                            "[MPC] e_c=%.3f, e_theta=%.3f, vx=%.3f, vy=%.3f, omega=%.3f, t=%.1fms",
                            frenet.e_c, frenet.e_theta, 
                            solution.vx_cmd, solution.vy_cmd, solution.omega_cmd,
                            solution.solve_time_ms);
                    }
                } else {
                    ROS_WARN_THROTTLE(1.0, "[OmniLocalPlannerROS] MPC solve failed: %s", 
                                      solution.status_msg.c_str());
                    double dt = control_rate_ > 1e-3 ? 1.0 / control_rate_ : mpc_params_.dt;
                    double vx = current_vx_;
                    double vy = current_vy_;
                    double decel = std::max(0.0, infeasible_decel_);
                    // 纵向减速
                    if (std::abs(vx) > 1e-3) {
                        double sign = vx >= 0.0 ? 1.0 : -1.0;
                        vx -= sign * decel * dt;
                        if (sign > 0.0) {
                            vx = std::max(vx, infeasible_min_speed_);
                            if (vx < 0.0) vx = 0.0;
                        } else {
                            vx = std::min(vx, -infeasible_min_speed_);
                            if (vx > 0.0) vx = 0.0;
                        }
                    } else {
                        vx = 0.0;
                    }
                    // 横向也减速到 0
                    if (std::abs(vy) > 1e-3) {
                        double sign_vy = vy >= 0.0 ? 1.0 : -1.0;
                        vy -= sign_vy * decel * dt;
                        if ((sign_vy > 0.0 && vy < 0.0) || (sign_vy < 0.0 && vy > 0.0)) {
                            vy = 0.0;
                        }
                    } else {
                        vy = 0.0;
                    }
                    double omega = current_omega_ * infeasible_omega_scale_;
                    publishCmdVel(vx, vy, omega);
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
            ROS_WARN("[OmniLocalPlannerROS] No odometry data");
        }
        return;
    }
    
    if (!path_handler_.isPathValid()) {
        if (state_ == PlannerState::TRACKING) {
            transitionTo(PlannerState::ERROR);
            ROS_WARN("[OmniLocalPlannerROS] Path invalid or timeout");
        }
        return;
    }
    
    // 检查是否到达目标
    if (state_ == PlannerState::TRACKING && path_handler_.isGoalReached()) {
        transitionTo(PlannerState::REACHED);
        resetWarmStart(false);
    }
}

void LocalPlannerROS::transitionTo(PlannerState new_state) {
    if (state_ != new_state) {
        ROS_INFO("[OmniLocalPlannerROS] State: %s -> %s",
                 plannerStateToString(state_).c_str(),
                 plannerStateToString(new_state).c_str());
        state_ = new_state;
    }
}

void LocalPlannerROS::publishCmdVel(double vx, double vy, double omega) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;     // 全向轮：使用 linear.y
    cmd.angular.z = omega;
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
            ROS_WARN_THROTTLE(1.0, "[OmniLocalPlannerROS] TF error in local_path: %s", ex.what());
            path.header.frame_id = base_frame_;
        }
    }
    
    if (refs.empty()) {
        return;
    }

    // 可视化起点：当前 base_link 原点
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

        // 还原到笛卡尔坐标（只使用横向误差 e_c）
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

void LocalPlannerROS::publishStatus() {
    if (status_pub_.getNumSubscribers() == 0) {
        return;
    }
    
    std_msgs::String msg;
    msg.data = plannerStateToString(state_);
    status_pub_.publish(msg);
}

void LocalPlannerROS::resetWarmStart(bool keep_u_prev) {
    mpc_solver_.resetWarmStart(keep_u_prev);
    if (!keep_u_prev) {
        last_control_.setZero();
    }
}

}  // namespace scout_omni_local_planner
