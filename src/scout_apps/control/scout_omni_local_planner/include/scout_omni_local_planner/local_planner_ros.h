/**
 * @file local_planner_ros.h
 * @brief 全向轮 MPC 局部规划器 ROS 接口
 * 
 * cmd_vel 使用 linear.x, linear.y, angular.z 三自由度
 */

#pragma once

#include "scout_omni_local_planner/types.h"
#include "scout_omni_local_planner/path_handler.h"
#include "scout_omni_local_planner/mpc_solver.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <mutex>

namespace scout_omni_local_planner {

class LocalPlannerROS {
public:
    LocalPlannerROS();
    ~LocalPlannerROS();
    
    bool initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    void run();

private:
    // 回调
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    // 控制循环
    void controlLoop(const ros::TimerEvent& event);
    
    // 辅助函数
    void loadParameters(ros::NodeHandle& pnh);
    void publishCmdVel(double vx, double vy, double omega);
    void publishLocalPath(const std::vector<StateVector>& predicted_states,
                          const std::vector<ReferencePoint>& refs);
    void publishSmoothedPath();
    void publishStatus();
    void updateState();
    void resetWarmStart(bool keep_u_prev);
    
    // 状态机
    void transitionTo(PlannerState new_state);
    
private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher smoothed_path_pub_;
    ros::Publisher status_pub_;
    ros::Timer control_timer_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 核心组件
    PathHandler path_handler_;
    MPCSolver mpc_solver_;
    
    // 参数
    MPCParams mpc_params_;
    VehicleParams vehicle_params_;
    PathHandlerParams path_params_;
    double control_rate_ = 20.0;
    std::string base_frame_ = "base_link";
    std::string map_frame_ = "map";
    double infeasible_decel_ = 1.0;
    double infeasible_omega_scale_ = 0.0;
    double infeasible_min_speed_ = 0.0;

    // 原地对齐模式
    bool heading_align_enable_ = false;
    double heading_align_enter_ = 0.8;
    double heading_align_exit_ = 0.4;
    double heading_align_omega_gain_ = 1.5;
    double heading_align_max_omega_ = 0.0;
    double heading_align_start_dist_ = 0.5;
    bool heading_align_active_ = false;
    
    // 状态
    PlannerState state_ = PlannerState::IDLE;
    geometry_msgs::PoseStamped current_pose_;
    double current_vx_ = 0.0;       // 纵向速度
    double current_vy_ = 0.0;       // 横向速度（全向轮新增）
    double current_omega_ = 0.0;
    ControlVector last_control_ = ControlVector::Zero();
    
    // 线程安全
    std::mutex mutex_;
    bool has_odom_ = false;
    bool has_path_ = false;
    
    bool verbose_ = false;
};

}  // namespace scout_omni_local_planner
