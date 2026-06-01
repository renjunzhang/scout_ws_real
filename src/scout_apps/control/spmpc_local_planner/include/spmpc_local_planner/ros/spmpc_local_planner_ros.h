#pragma once

#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/ros/diagnostics_publisher.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace spmpc_local_planner {

class SpmpcLocalPlannerROS {
public:
    SpmpcLocalPlannerROS();
    bool initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    void spin();

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void pathCallback(const nav_msgs::PathConstPtr& msg);
    void controlTimerCallback(const ros::TimerEvent&);
    RobotState robotStateFromOdom(const nav_msgs::Odometry& odom) const;
    bool robotStateFromLatest(RobotState& state);
    ReferencePath referencePathFromMsg(const nav_msgs::Path& path) const;
    void loadVariantOverrides(const std::string& variant_name);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer control_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    SpmpcProblem problem_;
    DiagnosticsPublisher diagnostics_;
    VariantConfig variant_;

    nav_msgs::Odometry last_odom_;
    bool have_odom_ = false;

    std::string odom_topic_ = "/odom";
    std::string path_topic_ = "/scout/global_path_fixed";
    std::string cmd_topic_ = "/cmd_vel";
    std::string robot_base_frame_ = "base_link";
    std::string experiment_mode_ = "fixed_path";
    bool publish_cmd_vel_ = true;
    bool use_tf_pose_ = true;
    double tf_timeout_sec_ = 0.05;
    double control_frequency_ = 20.0;
    double dt_ = 0.1;
    int horizon_steps_ = 30;
};

}  // namespace spmpc_local_planner
