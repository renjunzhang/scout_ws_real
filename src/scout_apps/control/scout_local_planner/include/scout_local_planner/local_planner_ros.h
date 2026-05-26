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
#include "scout_local_planner/profile_execution_cap.h"
#include "scout_local_planner/terminal_controller.h"
#include "scout_local_planner/slosh_feedback.h"

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

#include <limits>
#include <memory>
#include <mutex>
#include <vector>

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
    struct CostBreakdown {
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
    void publishReferencePath(const std::vector<ReferencePoint>& refs);
    void publishReferenceExecutionDebug(const std::vector<ReferencePoint>& refs);
    void publishSmoothedPath();
    void publishStatus();
    void publishSloshDebug(double solve_time_ms, bool solve_ok, bool publish_solver_debug = true);
    CostBreakdown computeCostBreakdown(const MPCSolution& solution,
                                       const std::vector<ReferencePoint>& refs,
                                       const MPCParams& params,
                                       const ControlVector& u_prev) const;
    void publishCostBreakdown(const CostBreakdown& breakdown);
    void publishSloshHorizonSummary(const MPCSolution& solution);
    void publishTerminalDebug();
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
    ros::Publisher reference_path_pub_;
    ros::Publisher smoothed_path_pub_;
    ros::Publisher status_pub_;
    ros::Timer control_timer_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 核心组件
    PathHandler path_handler_;
    MPCSolver mpc_solver_;
    ProfileExecutionCap profile_execution_cap_;
    TerminalController terminal_controller_;
    SloshFeedback slosh_feedback_;
    
    // 参数
    MPCParams mpc_params_;
    VehicleParams vehicle_params_;
    PathHandlerParams path_params_;
    double control_rate_ = 30.0;  // Hz
    std::string base_frame_ = "base_link";
    std::string map_frame_ = "map";
    double infeasible_decel_ = 1.0;       // 不可行时制动减速度 (m/s^2)
    double infeasible_omega_scale_ = 0.0; // 不可行时角速度缩放
    double infeasible_min_speed_ = 0.0;   // 不可行时线速度下限 (m/s)
    bool tracking_feasibility_guard_enable_ = true;
    int tracking_feas_fail_trigger_count_ = 3;
    int tracking_feas_fail_strong_trigger_count_ = 6;
    int tracking_feas_release_success_count_ = 5;
    double tracking_feas_v_cap_mild_ = 0.5;
    double tracking_feas_v_cap_strong_ = 0.3;
    double tracking_reentry_v_cap_ = 0.6;
    int tracking_reentry_ramp_steps_ = 10;
    // 执行层参考速度变化率限制：治理 v_des 突跳导致的纵向 ax 脉冲
    bool v_des_rate_limit_enable_ = true;
    double v_des_accel_limit_ = 0.6;
    double v_des_decel_limit_ = 0.8;
    int last_v_des_rate_limited_active_ = 0;
    double last_v_des_raw_ = 0.0;
    double last_v_des_target_ = 0.0;

    // 外部速度剖面执行层 cap 诊断（仅用于 TOPPRA/Ruckig-style baseline）
    int last_profile_cap_active_ = 0;
    double last_profile_cap_v_profile_ = std::numeric_limits<double>::quiet_NaN();
    double last_profile_cap_cmd_v_pre_ = std::numeric_limits<double>::quiet_NaN();
    double last_profile_cap_cmd_v_post_ = std::numeric_limits<double>::quiet_NaN();
    double last_profile_cap_implied_ax_ = std::numeric_limits<double>::quiet_NaN();
    double last_profile_cap_implied_jerk_ = std::numeric_limits<double>::quiet_NaN();

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

    ros::Time current_odom_time_;     // 当前 odom 时间戳
    SloshFeedbackOutput slosh_feedback_output_;

    double last_v_des_eff_ = 0.0;

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
    ros::Publisher terminal_recovery_latched_pub_;  // compatibility topic, always publishes 0
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

    // 实验 episode 标记
    int episode_id_ = 0;
    ros::Time reached_time_;
    double reached_debug_duration_ = 5.0;  // 到达终点后继续输出调试信息的时长
    double last_solve_time_ms_ = 0.0;
    bool last_solve_ok_ = false;
    double last_predicted_height_max_ = 0.0;
    int last_constraint_active_ = -1;  // -1=unknown, 0=below diagnostic height threshold, 1=above threshold
    std::string terminal_mode_debug_ = "NONE";
    GoalInfo terminal_goal_info_debug_;
    bool terminal_goal_info_valid_ = false;
    double last_terminal_v_envelope_ = std::numeric_limits<double>::infinity();
    int last_terminal_envelope_active_ = 0;
    int last_terminal_phase_active_ = 0;
    double last_terminal_cmd_v_pre_clamp_ = std::numeric_limits<double>::quiet_NaN();
    double last_terminal_cmd_v_post_clamp_ = std::numeric_limits<double>::quiet_NaN();
    int tracking_solve_fail_streak_ = 0;
    int tracking_solve_success_streak_ = 0;
    int tracking_reentry_ramp_steps_left_ = 0;
    bool tracking_feasibility_recovery_active_ = false;

    // cmd_vel 低通滤波（EMA）
    double filtered_v_ = 0.0;
    double filtered_omega_ = 0.0;
    double cmd_filter_alpha_v_ = 0.3;     // v 滤波系数，越大响应越快
    double cmd_filter_alpha_omega_ = 0.4; // omega 滤波系数
    double cmd_filter_kappa_boost_ = 0.5; // 曲率自适应增益：alpha += boost * |omega|
};

}  // namespace scout_local_planner
