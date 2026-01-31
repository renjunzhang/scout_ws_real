/**
 * @file local_planner_ros.cpp
 * @brief ROS 接口实现
 */

#include "scout_local_planner/local_planner_ros.h"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace scout_local_planner {

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
    pnh.param("mpc/Q_el", mpc_params_.Q_el, 1.0);
    pnh.param("mpc/Q_ec", mpc_params_.Q_ec, 10.0);
    pnh.param("mpc/Q_etheta", mpc_params_.Q_etheta, 5.0);
    pnh.param("mpc/Q_v", mpc_params_.Q_v, 1.0);
    pnh.param("mpc/R_a", mpc_params_.R_a, 1.0);
    pnh.param("mpc/R_alpha", mpc_params_.R_alpha, 1.0);
    pnh.param("mpc/R_da", mpc_params_.R_da, 0.1);
    pnh.param("mpc/R_dalpha", mpc_params_.R_dalpha, 0.1);
    pnh.param("mpc/Q_slosh", mpc_params_.Q_slosh, 0.0);
    
    // 车辆参数
    pnh.param("vehicle/v_max", vehicle_params_.v_max, 1.0);
    pnh.param("vehicle/v_min", vehicle_params_.v_min, -0.3);
    pnh.param("vehicle/omega_max", vehicle_params_.omega_max, 1.0);
    pnh.param("vehicle/a_max", vehicle_params_.a_max, 0.5);
    pnh.param("vehicle/alpha_max", vehicle_params_.alpha_max, 1.0);
    pnh.param("vehicle/track_width", vehicle_params_.track_width, 0.456);
    
    // 路径处理参数
    pnh.param("path_handler/lookahead_distance", path_params_.lookahead_distance, 1.0);
    pnh.param("path_handler/goal_tolerance", path_params_.goal_tolerance, 0.1);
    pnh.param("path_handler/yaw_tolerance", path_params_.yaw_tolerance, 0.1);
    pnh.param("path_handler/path_timeout", path_params_.path_timeout, 5.0);
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
    
    // 将 base_frame 传递给 path_handler
    path_params_.base_frame = base_frame_;
}

void LocalPlannerROS::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (path_handler_.updateGlobalPath(*msg)) {
        has_path_ = true;
        
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
    has_odom_ = true;
    
    // 更新位姿（从 odom 消息中提取）
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    
    // 更新 PathHandler 的机器人状态（关键！）
    path_handler_.updateRobotState(current_pose_, current_v_, current_omega_);
}

void LocalPlannerROS::controlLoop(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 更新状态
    updateState();
    
    // 发布状态
    publishStatus();
    
    // 根据状态机执行
    switch (state_) {
        case PlannerState::IDLE:
        case PlannerState::ERROR:
            // 停止
            publishCmdVel(0.0, 0.0);
            break;
            
        case PlannerState::REACHED:
            // 到达目标，停止
            publishCmdVel(0.0, 0.0);
            ROS_INFO_THROTTLE(5.0, "[LocalPlannerROS] Goal reached");
            break;
            
        case PlannerState::TRACKING:
            // 执行 MPC 控制
            {
                // 1. 获取参考点
                std::vector<ReferencePoint> ref_points;
                if (!path_handler_.getReferencePoints(
                        mpc_params_.N, mpc_params_.dt, 
                        vehicle_params_.v_max * 0.8,  // 期望速度
                        ref_points)) {
                    ROS_WARN_THROTTLE(1.0, "[LocalPlannerROS] Failed to get reference points");
                    publishCmdVel(0.0, 0.0);
                    return;
                }

                publishSmoothedPath();
                
                // 2. 获取 Frenet 误差
                FrenetState frenet;
                if (!path_handler_.getFrenetState(frenet)) {
                    ROS_WARN_THROTTLE(1.0, "[LocalPlannerROS] Failed to get Frenet state");
                    publishCmdVel(0.0, 0.0);
                    return;
                }
                
                // 3. 构建当前状态（避免初始状态越界导致不可行）
                auto clamp = [](double v, double lo, double hi) {
                    return std::max(lo, std::min(hi, v));
                };

                double v_clamped = clamp(current_v_, vehicle_params_.v_min, vehicle_params_.v_max);
                double omega_clamped = clamp(current_omega_, -vehicle_params_.omega_max, vehicle_params_.omega_max);

                StateVector current_state;
                current_state(StateIndex::E_L) = frenet.e_l;
                current_state(StateIndex::E_C) = frenet.e_c;
                current_state(StateIndex::E_THETA) = frenet.e_theta;
                current_state(StateIndex::V) = v_clamped;
                current_state(StateIndex::OMEGA) = omega_clamped;
                
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
                    publishLocalPath(solution.x_predicted);
                    
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
                    publishCmdVel(0.0, 0.0);
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
    
    // 检查是否到达目标
    if (state_ == PlannerState::TRACKING && path_handler_.isGoalReached()) {
        transitionTo(PlannerState::REACHED);
    }
}

void LocalPlannerROS::transitionTo(PlannerState new_state) {
    if (state_ != new_state) {
        ROS_INFO("[LocalPlannerROS] State: %s -> %s",
                 plannerStateToString(state_).c_str(),
                 plannerStateToString(new_state).c_str());
        state_ = new_state;
    }
}

void LocalPlannerROS::publishCmdVel(double v, double omega) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = v;
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

void LocalPlannerROS::publishLocalPath(const std::vector<StateVector>& predicted_states) {
    if (local_path_pub_.getNumSubscribers() == 0) {
        return;
    }
    
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = base_frame_;
    
    // 注意：这里预测的是 Frenet 误差，不是笛卡尔坐标
    // 简化处理：假设沿 x 方向
    double x = 0.0;
    for (size_t i = 0; i < predicted_states.size(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        
        // 使用速度积分位置（简化）
        if (i > 0) {
            x += predicted_states[i](StateIndex::V) * mpc_params_.dt;
        }
        pose.pose.position.x = x;
        pose.pose.position.y = predicted_states[i](StateIndex::E_C);  // 横向误差
        pose.pose.position.z = 0.0;
        
        path.poses.push_back(pose);
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

}  // namespace scout_local_planner
