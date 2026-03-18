/**
 * @file local_planner_ros.h
 * @brief ROS 接口
 * 
 * 独立节点模式的 MPC 局部规划器
 */

#pragma once

#include "scout_local_planner/types.h"
#include "scout_local_planner/path_handler.h"
#include "scout_local_planner/mpc_solver.h"
#include "scout_local_planner/slosh_integration.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <mutex>

namespace scout_local_planner {

class LocalPlannerROS {
public:
    LocalPlannerROS();
    ~LocalPlannerROS();
    
    /**
     * @brief 初始化
     */
    bool initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
    /**
     * @brief 主循环
     */
    void run();

private:
    // ====== 回调函数 ======
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
    // ====== 控制循环 ======
    void controlLoop(const ros::TimerEvent& event);
    
    // ====== 辅助函数 ======
    void loadParameters(ros::NodeHandle& pnh);
    void publishCmdVel(double v, double omega);
    void publishLocalPath(const std::vector<StateVector>& predicted_states,
                          const std::vector<ReferencePoint>& refs);
    void publishSmoothedPath();
    void publishStatus();
    void publishSloshDebug(double solve_time_ms, bool solve_ok);
    void updateSloshEstimate();
    double computePredictedSloshHeightMax(const MPCSolution& solution) const;
    void updateState();
    void resetWarmStart(bool keep_u_prev, bool reset_slosh = true);
    
    // ====== 状态机 ======
    void transitionTo(PlannerState new_state);
    
private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
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
    double control_rate_ = 20.0;  // Hz
    std::string base_frame_ = "base_link";
    std::string map_frame_ = "map";
    double infeasible_decel_ = 1.0;       // 不可行时制动减速度 (m/s^2)
    double infeasible_omega_scale_ = 0.0; // 不可行时角速度缩放
    double infeasible_min_speed_ = 0.0;   // 不可行时线速度下限 (m/s)

    // 原地对齐模式（heading align）
    bool heading_align_enable_ = false;
    double heading_align_enter_ = 0.8;   // 进入阈值 (rad)
    double heading_align_exit_ = 0.4;    // 退出阈值 (rad)
    double heading_align_omega_gain_ = 1.5;
    double heading_align_max_omega_ = 0.0; // <=0 表示使用 vehicle_params_.omega_max
    double heading_align_start_dist_ = 0.5; // 只在起点附近生效 (m)
    bool heading_align_active_ = false;
    
    // 状态
    PlannerState state_ = PlannerState::IDLE;
    geometry_msgs::PoseStamped current_pose_;
    double current_v_ = 0.0;
    double current_omega_ = 0.0;
    ControlVector last_control_ = ControlVector::Zero();
    
    // 线程安全
    std::mutex mutex_;
    bool has_odom_ = false;
    bool has_path_ = false;
    
    // 调试
    bool verbose_ = false;

    // ====== 液体晃动集成 (P0-A) ======
    SloshIntegration slosh_integration_;
    bool slosh_enabled_ = false;
    SloshParams slosh_params_;

    // 加速度估计（odom 差分 + EMA 低通）
    double prev_v_ = 0.0;
    double prev_omega_ = 0.0;
    ros::Time prev_odom_time_;        // 上一次 odom 时间戳
    ros::Time current_odom_time_;     // 当前 odom 时间戳
    bool has_prev_odom_ = false;
    double ax_filtered_ = 0.0;
    double ay_filtered_ = 0.0;      // odom 横向估计：v * omega（离心加速度近似）
    double alpha_filtered_ = 0.0;   // odom 角加速度估计
    double ay_est_used_ = 0.0;      // 实际注入 slosh 模型的横向加速度
    double alpha_est_used_ = 0.0;   // 实际注入 slosh 模型的角加速度
    double accel_filter_alpha_ = 0.3;  // EMA 滤波系数

    // IMU 接口预留（阶段 7）
    bool use_imu_lateral_accel_ = false;
    bool use_imu_yaw_rate_ = false;
    bool use_imu_alpha_z_ = false;
    std::string imu_topic_ = "/imu/data";
    double imu_filter_alpha_ = 0.3;
    bool has_imu_ = false;
    bool has_prev_imu_ = false;
    ros::Time prev_imu_time_;
    double imu_ay_filtered_ = 0.0;
    double imu_omega_z_filtered_ = 0.0;
    double imu_alpha_filtered_ = 0.0;
    double prev_imu_omega_z_ = 0.0;
    double omega_est_used_ = 0.0;     // 实际注入 slosh 模型的角速度

    // slosh-aware 速度治理（阶段 4）
    bool slosh_speed_governor_enable_ = false;
    double slosh_k_eta_ = 0.0;
    double slosh_ay_max_base_ = 0.0;
    double slosh_v_des_min_ = 0.0;
    double slosh_eta_deadband_ = 0.3;
    double slosh_eta_exit_ratio_ = 0.2;
    double slosh_preview_distance_ = 1.0;
    int slosh_min_active_steps_ = 10;
    bool slosh_governor_latched_ = false;
    int slosh_governor_hold_steps_ = 0;
    double last_v_des_eff_ = 0.0;
    int last_speed_governor_active_ = 0;

    // slosh 调试发布
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
    ros::Publisher slosh_speed_governor_active_pub_;
    ros::Publisher slosh_omega_est_used_pub_;
    ros::Publisher slosh_imu_omega_z_filtered_pub_;
    ros::Publisher mpc_solve_ms_pub_;
    ros::Publisher mpc_status_val_pub_;

    // 实验 episode 标记
    int episode_id_ = 0;
    ros::Time reached_time_;
    double reached_debug_duration_ = 5.0;  // 到达终点后继续输出调试信息的时长
    double last_solve_time_ms_ = 0.0;
    bool last_solve_ok_ = false;
    double last_predicted_height_max_ = 0.0;
    int last_constraint_active_ = -1;  // -1=unknown, 0=inactive, 1=active
    bool goal_stop_pending_ = false;   // 已进入目标容差区，先发 0 速制动，待车体实际减速后切 REACHED

    // cmd_vel 低通滤波（EMA）
    double filtered_v_ = 0.0;
    double filtered_omega_ = 0.0;
    double cmd_filter_alpha_v_ = 0.3;     // v 滤波系数，越大响应越快
    double cmd_filter_alpha_omega_ = 0.4; // omega 滤波系数
    double cmd_filter_kappa_boost_ = 0.5; // 曲率自适应增益：alpha += boost * |omega|
};

}  // namespace scout_local_planner
