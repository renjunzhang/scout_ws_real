#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <vector>

#include "lt_dwa_adapter/lt_dwa_planner.h"
#include "lt_dwa_adapter/trajectory_types.h"

namespace lt_dwa_adapter
{
class LtDwaAdapterROS
{
public:
  LtDwaAdapterROS();
  void spin();

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);

  bool transformPath(const nav_msgs::Path& input, nav_msgs::Path& output);
  bool transformPose(const geometry_msgs::PoseStamped& input, const std::string& target_frame,
                     geometry_msgs::PoseStamped& output) const;
  bool getRobotState(RobotState& state) const;
  bool goalCloseEnough(const RobotState& state) const;
  void publishCommand(const Command& command);
  void publishZeroCommand();
  void publishShadowCommand(const Command& command);
  void publishStatus(const std::string& status);
  void publishGlobalPlan();
  void publishLocalPlan(const TrajectoryCandidate& trajectory);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher shadow_cmd_pub_;
  ros::Publisher status_pub_;
  ros::Publisher global_plan_pub_;
  ros::Publisher local_plan_pub_;
  ros::Timer control_timer_;

  LtDwaPlanner planner_;
  PlannerConfig planner_config_;

  nav_msgs::Odometry latest_odom_;
  nav_msgs::OccupancyGrid latest_map_;
  nav_msgs::Path current_path_msg_;
  geometry_msgs::PoseStamped goal_;
  std::vector<Pose2D> current_path_;
  Command last_command_;
  bool have_last_command_ = false;
  std::string last_status_;

  std::string odom_topic_;
  std::string map_topic_;
  std::string global_path_topic_;
  std::string goal_topic_;
  std::string cmd_vel_topic_;
  std::string shadow_cmd_topic_;
  std::string status_topic_;
  std::string global_plan_topic_;
  std::string local_plan_topic_;
  std::string base_frame_;
  std::string plan_target_frame_;

  double controller_frequency_ = 10.0;
  double tf_timeout_sec_ = 0.2;
  bool publish_cmd_vel_ = false;
  bool publish_shadow_cmd_ = true;
  bool require_map_ = true;
  bool have_odom_ = false;
  bool have_map_ = false;
  bool have_path_ = false;
  bool have_goal_ = false;
};
}  // namespace lt_dwa_adapter
