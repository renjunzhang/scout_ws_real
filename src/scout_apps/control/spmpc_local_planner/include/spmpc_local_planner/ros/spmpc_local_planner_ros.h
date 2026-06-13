#pragma once

#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/reference/reference_path_preprocessor.h"
#include "spmpc_local_planner/ros/diagnostics_publisher.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cstddef>
#include <string>

namespace spmpc_local_planner {

class SpmpcLocalPlannerROS {
public:
    SpmpcLocalPlannerROS();
    bool initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    void spin();

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void pathCallback(const nav_msgs::PathConstPtr& msg);
    void costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
    void controlTimerCallback(const ros::TimerEvent&);
    void publishZeroCommand();
    void publishCommand(const geometry_msgs::Twist& desired);
    geometry_msgs::Twist applySharedCommandLimits(const geometry_msgs::Twist& desired,
                                                  const ros::Time& stamp,
                                                  geometry_msgs::Twist& previous,
                                                  double& dt,
                                                  bool& linear_limited,
                                                  bool& angular_rate_limited,
                                                  bool& angular_accel_limited);
    bool updateTerminalSpinFailGate(const SolverInput& input, const SolverOutput& output, double period_sec);
    void resetTerminalSpinFailGate();
    bool updateTrackingSafetyGate(const SolverInput& input,
                                  const SolverOutput& output,
                                  double period_sec,
                                  std::string& failure_status);
    void resetTrackingSafetyGate();
    RobotState robotStateFromOdom(const nav_msgs::Odometry& odom) const;
    bool robotStateFromLatest(RobotState& state);
    void updateSloshObserverFromOdom(const nav_msgs::Odometry& odom);
    bool updateReferenceSignature(const nav_msgs::Path& path);
    ReferencePath referencePathFromMsg(const nav_msgs::Path& path) const;
    CostmapGrid costmapFromMsg(const nav_msgs::OccupancyGrid& map) const;
    void loadVariantOverrides(const std::string& variant_name);
    SloshModelParams loadSloshParams() const;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber costmap_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer control_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    SpmpcProblem problem_;
    DiagnosticsPublisher diagnostics_;
    VariantConfig variant_;
    ReferencePathPreprocessor reference_preprocessor_;
    ReferencePathPreprocessParams reference_preprocess_params_;
    SloshDynamics slosh_observer_;
    SloshState current_slosh_;

    nav_msgs::Odometry last_odom_;
    nav_msgs::Odometry prev_odom_;
    bool have_odom_ = false;
    bool have_prev_odom_ = false;
    bool have_reference_signature_ = false;
    std::string reference_signature_frame_;
    std::size_t reference_signature_size_ = 0;
    double reference_signature_start_x_ = 0.0;
    double reference_signature_start_y_ = 0.0;
    double reference_signature_end_x_ = 0.0;
    double reference_signature_end_y_ = 0.0;

    std::string odom_topic_ = "/odom";
    std::string path_topic_ = "/scout/global_path_fixed";
    std::string costmap_topic_ = "/map";
    std::string cmd_topic_ = "/cmd_vel";
    std::string robot_base_frame_ = "base_link";
    std::string reference_target_frame_;
    std::string experiment_mode_ = "fixed_path";
    bool publish_cmd_vel_ = true;
    bool use_tf_pose_ = true;
    bool obstacle_enable_ = false;
    bool shared_cmd_linear_accel_limit_enable_ = true;
    double shared_cmd_linear_accel_max_ = 0.6;
    double shared_cmd_linear_accel_max_dt_ = 0.2;
    bool shared_cmd_angular_limit_enable_ = false;
    double shared_cmd_angular_rate_max_ = 1.2;
    double shared_cmd_angular_accel_max_ = 1.2;
    double shared_cmd_angular_accel_max_dt_ = 0.2;
    bool terminal_spin_fail_enable_ = true;
    double terminal_spin_fail_omega_threshold_ = 0.20;
    double terminal_spin_fail_max_duration_sec_ = 2.0;
    double terminal_spin_fail_duration_sec_ = 0.0;
    bool terminal_spin_fail_latched_ = false;
    bool tracking_safety_enable_ = true;
    bool tracking_safety_projection_enable_ = true;
    double tracking_safety_max_projection_distance_m_ = 0.50;
    double tracking_safety_max_projection_duration_sec_ = 0.20;
    double tracking_safety_projection_duration_sec_ = 0.0;
    bool tracking_safety_projection_latched_ = false;
    bool tracking_safety_spin_enable_ = true;
    double tracking_safety_spin_omega_threshold_ = 0.50;
    double tracking_safety_spin_max_duration_sec_ = 2.0;
    double tracking_safety_spin_duration_sec_ = 0.0;
    bool tracking_safety_spin_latched_ = false;
    geometry_msgs::Twist last_published_cmd_;
    ros::Time last_cmd_stamp_;
    bool have_last_published_cmd_ = false;
    double tf_timeout_sec_ = 0.05;
    double control_frequency_ = 30.0;
    double dt_ = 1.0 / 30.0;
    int horizon_steps_ = 60;
};

}  // namespace spmpc_local_planner
