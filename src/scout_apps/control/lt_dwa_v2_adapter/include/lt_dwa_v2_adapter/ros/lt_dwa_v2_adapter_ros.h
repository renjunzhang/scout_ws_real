#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <string>
#include <vector>

#include "lt_dwa_v2_adapter/core/planner_config.h"
#include "lt_dwa_v2_adapter/core/trajectory_types.h"
#include "lt_dwa_v2_adapter/geometry/planning_frames.h"
#include "lt_dwa_v2_adapter/search/lt_dwa_v2_planner.h"

namespace lt_dwa_v2_adapter
{
class LtDwaV2AdapterROS
{
public:
  LtDwaV2AdapterROS();
  void spin();

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void planningTimerCallback(const ros::TimerEvent& event);
  void commandPublishTimerCallback(const ros::TimerEvent& event);

  bool transformPath(const nav_msgs::Path& input, nav_msgs::Path& output);
  bool transformPose(const geometry_msgs::PoseStamped& input,
                     const std::string& target_frame,
                     geometry_msgs::PoseStamped& output) const;
  bool getPlanToMapTransform(const std::string& plan_frame,
                             const std::string& map_frame,
                             PlanningTransform2D& transform) const;
  std::string currentPlanFrame() const;
  bool getRobotState(RobotState& state) const;
  bool goalCloseEnough(const RobotState& state) const;
  Command clampCommand(const Command& command) const;
  Command commandForPublish(const ros::Time& now) const;
  void cacheCommand(const Command& command, const ros::Time& stamp, bool tracking_command);
  void cacheZeroCommand(const ros::Time& stamp);
  void publishCommand(const Command& command);
  void publishShadowCommand(const Command& command);
  void publishStatus(const std::string& status);
  void publishDiagnostics(const RobotState& state, const PlanResult& result);
  void publishGlobalPlan();
  void publishLocalPlan(const TrajectoryCandidate& trajectory);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::NodeHandle command_nh_;
  ros::CallbackQueue command_callback_queue_;
  mutable std::mutex command_mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher shadow_cmd_pub_;
  ros::Publisher status_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Publisher global_plan_pub_;
  ros::Publisher local_plan_pub_;
  ros::Timer planning_timer_;
  ros::Timer command_publish_timer_;

  PlannerConfig config_;
  LtDwaV2Planner planner_;

  nav_msgs::Odometry latest_odom_;
  nav_msgs::OccupancyGrid latest_map_;
  nav_msgs::Path current_path_msg_;
  geometry_msgs::PoseStamped goal_;
  std::vector<Pose2D> current_path_;
  Command last_command_;
  bool have_last_command_ = false;
  double last_progress_s_ = 0.0;
  bool have_last_progress_ = false;
  Command cached_command_;
  ros::Time cached_command_stamp_;
  bool have_cached_command_ = false;
  bool cached_command_tracking_ = false;
  std::string last_status_;

  bool have_odom_ = false;
  bool have_map_ = false;
  bool have_path_ = false;
  bool have_goal_ = false;
};
}  // namespace lt_dwa_v2_adapter
