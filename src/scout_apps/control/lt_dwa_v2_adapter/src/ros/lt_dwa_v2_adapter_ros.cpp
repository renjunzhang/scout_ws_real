#include "lt_dwa_v2_adapter/ros/lt_dwa_v2_adapter_ros.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "lt_dwa_v2_adapter/ros/diagnostics_formatter.h"
#include "lt_dwa_v2_adapter/ros/parameter_loader.h"

namespace lt_dwa_v2_adapter
{
namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(hi, value));
}

geometry_msgs::Quaternion yawToQuat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}
}  // namespace

LtDwaV2AdapterROS::LtDwaV2AdapterROS()
  : private_nh_("~")
  , tf_listener_(tf_buffer_)
{
  config_ = loadPlannerConfig(private_nh_);
  planner_.configure(config_);

  odom_sub_ = nh_.subscribe(config_.topics.odom_topic, 1, &LtDwaV2AdapterROS::odomCallback, this);
  map_sub_ = nh_.subscribe(config_.topics.map_topic, 1, &LtDwaV2AdapterROS::mapCallback, this);
  path_sub_ = nh_.subscribe(config_.topics.global_path_topic, 1, &LtDwaV2AdapterROS::pathCallback, this);
  goal_sub_ = nh_.subscribe(config_.topics.goal_topic, 1, &LtDwaV2AdapterROS::goalCallback, this);

  if (config_.runtime.publish_cmd_vel)
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(config_.topics.cmd_vel_topic, 1);
  if (config_.runtime.publish_shadow_cmd)
    shadow_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(config_.topics.shadow_cmd_topic, 1);
  status_pub_ = nh_.advertise<std_msgs::String>(config_.topics.status_topic, 1, true);
  diagnostics_pub_ = nh_.advertise<std_msgs::String>(config_.topics.diagnostics_topic, 1, false);
  global_plan_pub_ = nh_.advertise<nav_msgs::Path>(config_.topics.global_plan_topic, 1, true);
  local_plan_pub_ = nh_.advertise<nav_msgs::Path>(config_.topics.local_plan_topic, 1, true);

  command_nh_.setCallbackQueue(&command_callback_queue_);
  planning_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, config_.timing.planning_frequency)),
                                    &LtDwaV2AdapterROS::planningTimerCallback, this);
  command_publish_timer_ = command_nh_.createTimer(
      ros::Duration(1.0 / std::max(1.0, config_.timing.command_publish_frequency)),
      &LtDwaV2AdapterROS::commandPublishTimerCallback, this);

  cacheZeroCommand(ros::Time::now());
  publishStatus("INITIALIZED");
  ROS_INFO_STREAM("[lt_dwa_v2_adapter] initialized shadow_only=" << (!config_.runtime.publish_cmd_vel)
                  << " publish_cmd_vel=" << config_.runtime.publish_cmd_vel
                  << " cmd_vel_topic=" << config_.topics.cmd_vel_topic
                  << " shadow_cmd_topic=" << config_.topics.shadow_cmd_topic
                  << " path_topic=" << config_.topics.global_path_topic
                  << " map_topic=" << config_.topics.map_topic
                  << " planning_frequency=" << config_.timing.planning_frequency
                  << " command_publish_frequency=" << config_.timing.command_publish_frequency
                  << " command_stale_timeout_sec=" << config_.timing.command_stale_timeout_sec
                  << " require_map=" << config_.runtime.require_map);
}

void LtDwaV2AdapterROS::spin()
{
  ros::AsyncSpinner command_spinner(1, &command_callback_queue_);
  command_spinner.start();
  ros::spin();
  command_spinner.stop();
}

void LtDwaV2AdapterROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  latest_odom_ = *msg;
  have_odom_ = true;
}

void LtDwaV2AdapterROS::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  latest_map_ = *msg;
  have_map_ = true;
}

void LtDwaV2AdapterROS::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  if (!msg || msg->poses.empty())
    return;

  nav_msgs::Path transformed;
  if (!transformPath(*msg, transformed))
  {
    publishStatus("TF_ERROR");
    return;
  }

  current_path_msg_ = transformed;
  current_path_.clear();
  current_path_.reserve(transformed.poses.size());
  for (size_t i = 0; i < transformed.poses.size(); ++i)
  {
    const auto& pose = transformed.poses[i];
    Pose2D p;
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    if (i + 1 < transformed.poses.size())
    {
      const auto& next = transformed.poses[i + 1];
      const double dx = next.pose.position.x - pose.pose.position.x;
      const double dy = next.pose.position.y - pose.pose.position.y;
      p.yaw = (std::abs(dx) + std::abs(dy) > 1e-9) ? std::atan2(dy, dx) : tf2::getYaw(pose.pose.orientation);
    }
    else
    {
      p.yaw = tf2::getYaw(pose.pose.orientation);
    }
    current_path_.push_back(p);
  }

  goal_ = transformed.poses.back();
  have_goal_ = true;
  have_path_ = true;
  last_progress_s_ = 0.0;
  have_last_progress_ = false;
  publishGlobalPlan();
  publishStatus("PATH_RECEIVED");
}

void LtDwaV2AdapterROS::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (!msg)
    return;
  const std::string target_frame = current_path_msg_.header.frame_id.empty() ?
                                       currentPlanFrame() :
                                       current_path_msg_.header.frame_id;
  geometry_msgs::PoseStamped transformed;
  if (!transformPose(*msg, target_frame, transformed))
  {
    publishStatus("TF_ERROR");
    return;
  }
  goal_ = transformed;
  have_goal_ = true;
  publishStatus("GOAL_RECEIVED");
}

bool LtDwaV2AdapterROS::transformPath(const nav_msgs::Path& input, nav_msgs::Path& output)
{
  const std::string input_frame = input.header.frame_id.empty() ? input.poses.front().header.frame_id : input.header.frame_id;
  const std::string source_frame = input_frame.empty() ? std::string("map") : input_frame;
  const std::string target_frame = config_.frames.plan_target_frame.empty() ? source_frame : config_.frames.plan_target_frame;
  output.header = input.header;
  output.header.frame_id = target_frame;
  output.header.stamp = ros::Time(0);
  output.poses.clear();
  output.poses.reserve(input.poses.size());
  for (auto pose : input.poses)
  {
    if (pose.header.frame_id.empty())
      pose.header.frame_id = source_frame;
    pose.header.stamp = ros::Time(0);
    if (pose.header.frame_id == target_frame)
    {
      output.poses.push_back(pose);
      output.poses.back().header.frame_id = target_frame;
      continue;
    }
    geometry_msgs::PoseStamped transformed;
    if (!transformPose(pose, target_frame, transformed))
      return false;
    output.poses.push_back(transformed);
  }
  return true;
}

bool LtDwaV2AdapterROS::transformPose(const geometry_msgs::PoseStamped& input,
                                      const std::string& target_frame,
                                      geometry_msgs::PoseStamped& output) const
{
  if (target_frame.empty())
  {
    output = input;
    return true;
  }
  geometry_msgs::PoseStamped stamped = input;
  if (stamped.header.frame_id.empty())
    stamped.header.frame_id = "map";
  stamped.header.stamp = ros::Time(0);
  if (stamped.header.frame_id == target_frame)
  {
    output = stamped;
    return true;
  }
  try
  {
    output = tf_buffer_.transform(stamped, target_frame, ros::Duration(config_.frames.tf_timeout_sec));
    output.header.stamp = ros::Time(0);
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(2.0, "[lt_dwa_v2_adapter] transform %s -> %s failed: %s",
                      stamped.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

bool LtDwaV2AdapterROS::getPlanToMapTransform(const std::string& plan_frame,
                                              const std::string& map_frame,
                                              PlanningTransform2D& transform) const
{
  transform = identityTransform(plan_frame, map_frame);
  if (map_frame.empty() || plan_frame.empty() || plan_frame == map_frame)
    return true;

  try
  {
    const geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(map_frame, plan_frame, ros::Time(0),
                                                                          ros::Duration(config_.frames.tf_timeout_sec));
    transform.valid = true;
    transform.source_frame = plan_frame;
    transform.target_frame = map_frame;
    transform.x = tf.transform.translation.x;
    transform.y = tf.transform.translation.y;
    transform.yaw = tf2::getYaw(tf.transform.rotation);
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(2.0, "[lt_dwa_v2_adapter] plan/map transform %s -> %s failed: %s",
                      plan_frame.c_str(), map_frame.c_str(), ex.what());
    transform.valid = false;
    transform.source_frame = plan_frame;
    transform.target_frame = map_frame;
    return false;
  }
}

std::string LtDwaV2AdapterROS::currentPlanFrame() const
{
  if (!current_path_msg_.header.frame_id.empty())
    return current_path_msg_.header.frame_id;
  if (!config_.frames.plan_target_frame.empty())
    return config_.frames.plan_target_frame;
  return "map";
}

bool LtDwaV2AdapterROS::getRobotState(RobotState& state) const
{
  const std::string target_frame = currentPlanFrame();
  geometry_msgs::PoseStamped base_pose;
  base_pose.header.frame_id = config_.frames.base_frame;
  base_pose.header.stamp = ros::Time(0);
  base_pose.pose.orientation.w = 1.0;
  geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(base_pose, target_frame, robot_pose))
  {
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header = latest_odom_.header;
    odom_pose.pose = latest_odom_.pose.pose;
    if (!transformPose(odom_pose, target_frame, robot_pose))
      return false;
  }

  state.x = robot_pose.pose.position.x;
  state.y = robot_pose.pose.position.y;
  state.yaw = tf2::getYaw(robot_pose.pose.orientation);
  state.v = latest_odom_.twist.twist.linear.x;
  state.omega = latest_odom_.twist.twist.angular.z;
  return true;
}

bool LtDwaV2AdapterROS::goalCloseEnough(const RobotState& state) const
{
  if (!have_goal_)
    return false;
  const double dx = goal_.pose.position.x - state.x;
  const double dy = goal_.pose.position.y - state.y;
  const double dist = std::hypot(dx, dy);
  const double yaw_err = std::abs(normalizeAngle(tf2::getYaw(goal_.pose.orientation) - state.yaw));
  return dist <= config_.goal.xy_tolerance_m && yaw_err <= config_.goal.yaw_tolerance_rad;
}

Command LtDwaV2AdapterROS::clampCommand(const Command& command) const
{
  Command clamped;
  const double min_v = config_.limits.allow_reverse ? -config_.limits.v_max_mps : 0.0;
  clamped.v = clamp(command.v, min_v, config_.limits.v_max_mps);
  clamped.omega = clamp(command.omega, -config_.limits.omega_max_radps, config_.limits.omega_max_radps);
  return clamped;
}

Command LtDwaV2AdapterROS::commandForPublish(const ros::Time& now) const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!have_cached_command_ || !cached_command_tracking_ || cached_command_stamp_.isZero())
    return Command{};
  if ((now - cached_command_stamp_).toSec() > config_.timing.command_stale_timeout_sec)
    return Command{};
  return cached_command_;
}

void LtDwaV2AdapterROS::cacheCommand(const Command& command, const ros::Time& stamp, bool tracking_command)
{
  const Command clamped = clampCommand(command);
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    cached_command_ = clamped;
    cached_command_stamp_ = stamp;
    cached_command_tracking_ = tracking_command;
    have_cached_command_ = true;
  }

  last_command_ = clamped;
  have_last_command_ = true;
}

void LtDwaV2AdapterROS::cacheZeroCommand(const ros::Time& stamp)
{
  cacheCommand(Command{}, stamp, false);
}

void LtDwaV2AdapterROS::planningTimerCallback(const ros::TimerEvent&)
{
  const ros::Time now = ros::Time::now();
  if (!have_odom_)
  {
    cacheZeroCommand(now);
    publishStatus(statusString(PlannerStatusCode::WaitingForOdom));
    return;
  }
  if (!have_path_ || current_path_.empty())
  {
    cacheZeroCommand(now);
    publishStatus(statusString(PlannerStatusCode::WaitingForPath));
    return;
  }
  if (config_.runtime.require_map && !have_map_)
  {
    cacheZeroCommand(now);
    publishStatus(statusString(PlannerStatusCode::WaitingForMap));
    return;
  }

  RobotState state;
  if (!getRobotState(state))
  {
    cacheZeroCommand(now);
    publishStatus("TF_ERROR");
    return;
  }
  if (goalCloseEnough(state))
  {
    cacheZeroCommand(now);
    publishLocalPlan(TrajectoryCandidate{});
    publishStatus(statusString(PlannerStatusCode::GoalReached));
    return;
  }

  const nav_msgs::OccupancyGrid* map = have_map_ ? &latest_map_ : nullptr;
  const std::string plan_frame = currentPlanFrame();
  const std::string map_frame = map && !latest_map_.header.frame_id.empty() ? latest_map_.header.frame_id : std::string();
  PlanningTransform2D plan_to_map = identityTransform(plan_frame, map_frame);
  bool plan_map_transform_ok = true;
  if (map)
  {
    plan_map_transform_ok = getPlanToMapTransform(plan_frame, map_frame, plan_to_map);
    if (!plan_map_transform_ok && config_.runtime.require_map)
    {
      cacheZeroCommand(now);
      publishLocalPlan(TrajectoryCandidate{});
      publishStatus(statusString(PlannerStatusCode::MapTfError));
      return;
    }
  }

  OccupancyAdapter occupancy_adapter(map && plan_map_transform_ok ? map : nullptr, plan_to_map, config_.occupancy);
  const double min_progress_s = have_last_progress_ ?
                                    std::max(0.0, last_progress_s_ - config_.tracking.progress_rollback_tolerance_m) :
                                    0.0;
  const double max_progress_s = have_last_progress_ ?
                                    last_progress_s_ + config_.tracking.max_progress_advance_per_step_m :
                                    config_.tracking.lookahead_distance_m;
  PlanResult result = planner_.plan(state, current_path_, occupancy_adapter.hasGrid() ? &occupancy_adapter : nullptr,
                                    min_progress_s, max_progress_s);
  result.diagnostics.plan_map_transform_ok = plan_map_transform_ok;
  if (result.diagnostics.has_initial_match &&
      (result.status_code == PlannerStatusCode::Tracking || result.status_code == PlannerStatusCode::GoalReached))
  {
    last_progress_s_ = have_last_progress_ ? std::max(last_progress_s_, result.diagnostics.initial_progress_s) :
                                             result.diagnostics.initial_progress_s;
    have_last_progress_ = true;
  }

  const ros::Time result_stamp = ros::Time::now();
  publishLocalPlan(result.best);
  publishDiagnostics(state, result);
  if (result.valid && result.status_code == PlannerStatusCode::Tracking)
    cacheCommand(result.command, result_stamp, true);
  else
    cacheZeroCommand(result_stamp);

  std::string status = formatStatus(result);
  if (have_goal_)
  {
    const double dx = goal_.pose.position.x - state.x;
    const double dy = goal_.pose.position.y - state.y;
    const double yaw_err = std::abs(normalizeAngle(tf2::getYaw(goal_.pose.orientation) - state.yaw));
    std::ostringstream ss;
    ss << status << " goal_dist=" << std::hypot(dx, dy) << " goal_yaw_err=" << yaw_err;
    status = ss.str();
  }
  publishStatus(status);
}

void LtDwaV2AdapterROS::commandPublishTimerCallback(const ros::TimerEvent&)
{
  const Command command = commandForPublish(ros::Time::now());
  publishShadowCommand(command);
  publishCommand(command);
}

void LtDwaV2AdapterROS::publishCommand(const Command& command)
{
  if (!config_.runtime.publish_cmd_vel || !cmd_pub_)
    return;
  const Command clamped = clampCommand(command);
  geometry_msgs::Twist msg;
  msg.linear.x = clamped.v;
  msg.angular.z = clamped.omega;
  cmd_pub_.publish(msg);
}

void LtDwaV2AdapterROS::publishShadowCommand(const Command& command)
{
  if (!config_.runtime.publish_shadow_cmd || !shadow_cmd_pub_)
    return;
  const Command clamped = clampCommand(command);
  geometry_msgs::Twist msg;
  msg.linear.x = clamped.v;
  msg.angular.z = clamped.omega;
  shadow_cmd_pub_.publish(msg);
}

void LtDwaV2AdapterROS::publishStatus(const std::string& status)
{
  if (status == last_status_)
    return;
  last_status_ = status;
  std_msgs::String msg;
  msg.data = status;
  status_pub_.publish(msg);
}

void LtDwaV2AdapterROS::publishDiagnostics(const RobotState& state, const PlanResult& result)
{
  if (!config_.runtime.publish_diagnostics)
    return;

  RosDiagnosticsContext context;
  context.plan_frame = currentPlanFrame();
  context.map_frame = have_map_ && !latest_map_.header.frame_id.empty() ? latest_map_.header.frame_id : std::string();
  context.raw_odom_frame = latest_odom_.header.frame_id;
  context.raw_odom_child_frame = latest_odom_.child_frame_id;
  context.path_size = current_path_.size();
  context.have_last_progress = have_last_progress_;
  context.tracker_progress_s = last_progress_s_;
  context.state = state;
  context.raw_odom_x = latest_odom_.pose.pose.position.x;
  context.raw_odom_y = latest_odom_.pose.pose.position.y;
  context.raw_odom_yaw = tf2::getYaw(latest_odom_.pose.pose.orientation);

  std_msgs::String msg;
  msg.data = formatDiagnostics(context, result);
  diagnostics_pub_.publish(msg);
}

void LtDwaV2AdapterROS::publishGlobalPlan()
{
  if (!have_path_ && current_path_msg_.poses.empty())
    return;
  global_plan_pub_.publish(current_path_msg_);
}

void LtDwaV2AdapterROS::publishLocalPlan(const TrajectoryCandidate& trajectory)
{
  nav_msgs::Path path;
  path.header.frame_id = current_path_msg_.header.frame_id.empty() ? std::string("map") : current_path_msg_.header.frame_id;
  path.header.stamp = ros::Time::now();
  path.poses.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.state.x;
    pose.pose.position.y = point.state.y;
    pose.pose.orientation = yawToQuat(point.state.yaw);
    path.poses.push_back(pose);
  }
  local_plan_pub_.publish(path);
}
}  // namespace lt_dwa_v2_adapter
