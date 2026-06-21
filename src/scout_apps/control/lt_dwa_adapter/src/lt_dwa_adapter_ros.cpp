#include "lt_dwa_adapter/lt_dwa_adapter_ros.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace lt_dwa_adapter
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

LtDwaAdapterROS::LtDwaAdapterROS()
  : private_nh_("~")
  , tf_listener_(tf_buffer_)
{
  private_nh_.param("odom_topic", odom_topic_, std::string("/odom"));
  private_nh_.param("map_topic", map_topic_, std::string("/map"));
  private_nh_.param("global_path_topic", global_path_topic_, std::string("/scout/global_path_fixed"));
  private_nh_.param("goal_topic", goal_topic_, std::string("/scout/goal"));
  private_nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/lt_dwa/shadow_cmd_vel"));
  private_nh_.param("shadow_cmd_topic", shadow_cmd_topic_, std::string("/baseline/lt_dwa/shadow_cmd_vel"));
  private_nh_.param("status_topic", status_topic_, std::string("/baseline/lt_dwa/status"));
  private_nh_.param("diagnostics_topic", diagnostics_topic_, std::string("/baseline/lt_dwa/diagnostics"));
  private_nh_.param("global_plan_topic", global_plan_topic_, std::string("/baseline/lt_dwa/global_plan"));
  private_nh_.param("local_plan_topic", local_plan_topic_, std::string("/baseline/lt_dwa/local_plan"));
  private_nh_.param("base_frame", base_frame_, std::string("base_footprint"));
  private_nh_.param("plan_target_frame", plan_target_frame_, std::string(""));
  double controller_frequency = 10.0;
  private_nh_.param("controller_frequency", controller_frequency, 10.0);
  private_nh_.param("planning_frequency", planning_frequency_, controller_frequency);
  private_nh_.param("command_publish_frequency", command_publish_frequency_, 25.0);
  private_nh_.param("command_stale_timeout_sec", command_stale_timeout_sec_, 2.0);
  private_nh_.param("tf_timeout_sec", tf_timeout_sec_, 0.2);
  private_nh_.param("publish_cmd_vel", publish_cmd_vel_, false);
  private_nh_.param("publish_shadow_cmd", publish_shadow_cmd_, true);
  private_nh_.param("publish_diagnostics", publish_diagnostics_, false);
  private_nh_.param("require_map", require_map_, true);

  if (!std::isfinite(planning_frequency_) || planning_frequency_ <= 0.0)
    planning_frequency_ = 10.0;
  if (!std::isfinite(command_publish_frequency_) || command_publish_frequency_ <= 0.0)
    command_publish_frequency_ = 25.0;
  if (!std::isfinite(command_stale_timeout_sec_) || command_stale_timeout_sec_ <= 0.0)
    command_stale_timeout_sec_ = 2.0;

  private_nh_.param("v_max_mps", planner_config_.limits.v_max_mps, 0.8);
  private_nh_.param("omega_max_radps", planner_config_.limits.omega_max_radps, 1.2);
  private_nh_.param("a_max_mps2", planner_config_.limits.a_max_mps2, 0.6);
  private_nh_.param("alpha_max_radps2", planner_config_.limits.alpha_max_radps2, 1.2);
  private_nh_.param("allow_reverse", planner_config_.limits.allow_reverse, false);
  private_nh_.param("dt", planner_config_.dt, 0.15);
  private_nh_.param("horizon_steps", planner_config_.horizon_steps, 12);
  private_nh_.param("v_samples", planner_config_.v_samples, 7);
  private_nh_.param("omega_samples", planner_config_.omega_samples, 9);
  private_nh_.param("top_k_per_layer", planner_config_.top_k_per_layer, 80);
  private_nh_.param("robot_radius_m", planner_config_.robot_radius_m, 0.35);
  private_nh_.param("clearance_radius_m", planner_config_.clearance_radius_m, 0.80);
  private_nh_.param("lethal_occupancy", planner_config_.lethal_occupancy, 65);
  private_nh_.param("treat_unknown_as_occupied", planner_config_.treat_unknown_as_occupied, false);
  private_nh_.param("xy_goal_tolerance", planner_config_.goal_xy_tolerance_m, 0.20);
  private_nh_.param("yaw_goal_tolerance", planner_config_.goal_yaw_tolerance_rad, 0.30);
  private_nh_.param("max_tracking_deviation_m", planner_config_.max_tracking_deviation_m, 1.50);
  private_nh_.param("lookahead_distance_m", planner_config_.lookahead_distance_m, 0.55);
  private_nh_.param("progress_rollback_tolerance_m", planner_config_.progress_rollback_tolerance_m, 0.35);
  private_nh_.param("max_progress_advance_per_step_m", planner_config_.max_progress_advance_per_step_m, 0.35);
  private_nh_.param("cross_track_heading_gain", planner_config_.cross_track_heading_gain, 1.40);
  private_nh_.param("tracking_slowdown_lateral_m", planner_config_.tracking_slowdown_lateral_m, 0.35);
  private_nh_.param("tracking_slowdown_heading_rad", planner_config_.tracking_slowdown_heading_rad, 0.65);
  private_nh_.param("weights/obstacle", planner_config_.weights.obstacle, 2.0);
  private_nh_.param("weights/path_lateral", planner_config_.weights.path_lateral, 4.0);
  private_nh_.param("weights/heading", planner_config_.weights.heading, 2.0);
  private_nh_.param("weights/progress", planner_config_.weights.progress, 3.0);
  private_nh_.param("weights/terminal", planner_config_.weights.terminal, 4.0);
  private_nh_.param("weights/smooth_v", planner_config_.weights.smooth_v, 0.0);
  private_nh_.param("weights/smooth_omega", planner_config_.weights.smooth_omega, 0.08);
  private_nh_.param("weights/speed", planner_config_.weights.speed, 1.0);

  planner_.configure(planner_config_);

  odom_sub_ = nh_.subscribe(odom_topic_, 1, &LtDwaAdapterROS::odomCallback, this);
  map_sub_ = nh_.subscribe(map_topic_, 1, &LtDwaAdapterROS::mapCallback, this);
  path_sub_ = nh_.subscribe(global_path_topic_, 1, &LtDwaAdapterROS::pathCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic_, 1, &LtDwaAdapterROS::goalCallback, this);

  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
  shadow_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(shadow_cmd_topic_, 1);
  status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
  diagnostics_pub_ = nh_.advertise<std_msgs::String>(diagnostics_topic_, 1, false);
  global_plan_pub_ = nh_.advertise<nav_msgs::Path>(global_plan_topic_, 1, true);
  local_plan_pub_ = nh_.advertise<nav_msgs::Path>(local_plan_topic_, 1, true);

  command_nh_.setCallbackQueue(&command_callback_queue_);
  const double planning_period = 1.0 / std::max(1.0, planning_frequency_);
  const double command_publish_period = 1.0 / std::max(1.0, command_publish_frequency_);
  planning_timer_ = nh_.createTimer(ros::Duration(planning_period), &LtDwaAdapterROS::planningTimerCallback, this);
  command_publish_timer_ = command_nh_.createTimer(ros::Duration(command_publish_period),
                                                   &LtDwaAdapterROS::commandPublishTimerCallback, this);

  cacheZeroCommand(ros::Time::now());
  publishStatus("INITIALIZED");
  ROS_INFO_STREAM("[lt_dwa_adapter] initialized shadow=" << (publish_cmd_vel_ ? "false" : "true")
                  << " publish_cmd_vel=" << publish_cmd_vel_ << " cmd_vel_topic=" << cmd_vel_topic_
                  << " path_topic=" << global_path_topic_ << " map_topic=" << map_topic_
                  << " diagnostics=" << publish_diagnostics_ << " diagnostics_topic=" << diagnostics_topic_
                  << " planning_frequency=" << planning_frequency_
                  << " command_publish_frequency=" << command_publish_frequency_
                  << " command_stale_timeout_sec=" << command_stale_timeout_sec_
                  << " max_tracking_deviation_m=" << planner_config_.max_tracking_deviation_m
                  << " lookahead_distance_m=" << planner_config_.lookahead_distance_m
                  << " progress_rollback_tolerance_m=" << planner_config_.progress_rollback_tolerance_m
                  << " max_progress_advance_per_step_m=" << planner_config_.max_progress_advance_per_step_m
                  << " cross_track_heading_gain=" << planner_config_.cross_track_heading_gain
                  << " tracking_slowdown_lateral_m=" << planner_config_.tracking_slowdown_lateral_m
                  << " tracking_slowdown_heading_rad=" << planner_config_.tracking_slowdown_heading_rad);
}

void LtDwaAdapterROS::spin()
{
  ros::AsyncSpinner command_spinner(1, &command_callback_queue_);
  command_spinner.start();
  ros::spin();
  command_spinner.stop();
}

void LtDwaAdapterROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  latest_odom_ = *msg;
  have_odom_ = true;
}

void LtDwaAdapterROS::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  latest_map_ = *msg;
  have_map_ = true;
}

void LtDwaAdapterROS::pathCallback(const nav_msgs::Path::ConstPtr& msg)
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

void LtDwaAdapterROS::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (!msg)
    return;
  const std::string target_frame = current_path_msg_.header.frame_id.empty() ? plan_target_frame_ : current_path_msg_.header.frame_id;
  if (!target_frame.empty())
  {
    geometry_msgs::PoseStamped transformed;
    if (!transformPose(*msg, target_frame, transformed))
    {
      publishStatus("TF_ERROR");
      return;
    }
    goal_ = transformed;
  }
  else
  {
    goal_ = *msg;
  }
  have_goal_ = true;
  publishStatus("GOAL_RECEIVED");
}

bool LtDwaAdapterROS::transformPath(const nav_msgs::Path& input, nav_msgs::Path& output)
{
  const std::string input_frame = input.header.frame_id.empty() ? input.poses.front().header.frame_id : input.header.frame_id;
  const std::string source_frame = input_frame.empty() ? std::string("map") : input_frame;
  const std::string target_frame = plan_target_frame_.empty() ? source_frame : plan_target_frame_;
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

bool LtDwaAdapterROS::transformPose(const geometry_msgs::PoseStamped& input,
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
    output = tf_buffer_.transform(stamped, target_frame, ros::Duration(tf_timeout_sec_));
    output.header.stamp = ros::Time(0);
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(2.0, "[lt_dwa_adapter] transform %s -> %s failed: %s",
                      stamped.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

bool LtDwaAdapterROS::getPlanToMapTransform(const std::string& plan_frame,
                                            const std::string& map_frame,
                                            PlanningTransform2D& transform) const
{
  transform = identityTransform(plan_frame, map_frame);
  if (map_frame.empty() || plan_frame.empty() || plan_frame == map_frame)
    return true;

  try
  {
    const geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(map_frame, plan_frame, ros::Time(0),
                                                                          ros::Duration(tf_timeout_sec_));
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
    ROS_WARN_THROTTLE(2.0, "[lt_dwa_adapter] plan/map transform %s -> %s failed: %s",
                      plan_frame.c_str(), map_frame.c_str(), ex.what());
    transform.valid = false;
    transform.source_frame = plan_frame;
    transform.target_frame = map_frame;
    return false;
  }
}

std::string LtDwaAdapterROS::currentPlanFrame() const
{
  if (!current_path_msg_.header.frame_id.empty())
    return current_path_msg_.header.frame_id;
  if (!plan_target_frame_.empty())
    return plan_target_frame_;
  return "map";
}

bool LtDwaAdapterROS::getRobotState(RobotState& state) const
{
  const std::string target_frame = currentPlanFrame();
  geometry_msgs::PoseStamped base_pose;
  base_pose.header.frame_id = base_frame_;
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

bool LtDwaAdapterROS::goalCloseEnough(const RobotState& state) const
{
  if (!have_goal_)
    return false;
  const double dx = goal_.pose.position.x - state.x;
  const double dy = goal_.pose.position.y - state.y;
  const double dist = std::hypot(dx, dy);
  const double yaw_err = std::abs(normalizeAngle(tf2::getYaw(goal_.pose.orientation) - state.yaw));
  return dist <= planner_config_.goal_xy_tolerance_m && yaw_err <= planner_config_.goal_yaw_tolerance_rad;
}

Command LtDwaAdapterROS::clampCommand(const Command& command) const
{
  Command clamped;
  const double min_v = planner_config_.limits.allow_reverse ? -planner_config_.limits.v_max_mps : 0.0;
  clamped.v = clamp(command.v, min_v, planner_config_.limits.v_max_mps);
  clamped.omega = clamp(command.omega, -planner_config_.limits.omega_max_radps, planner_config_.limits.omega_max_radps);
  return clamped;
}

bool LtDwaAdapterROS::cachedCommandFresh(const ros::Time& now) const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!have_cached_command_ || cached_command_stamp_.isZero())
    return false;
  return (now - cached_command_stamp_).toSec() <= command_stale_timeout_sec_;
}

Command LtDwaAdapterROS::commandForPublish(const ros::Time& now) const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!have_cached_command_ || !cached_command_tracking_ || cached_command_stamp_.isZero())
    return Command{};
  if ((now - cached_command_stamp_).toSec() > command_stale_timeout_sec_)
    return Command{};
  return cached_command_;
}

void LtDwaAdapterROS::cacheCommand(const Command& command, const ros::Time& stamp, bool tracking_command)
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

void LtDwaAdapterROS::cacheZeroCommand(const ros::Time& stamp)
{
  cacheCommand(Command{}, stamp, false);
}

void LtDwaAdapterROS::planningTimerCallback(const ros::TimerEvent&)
{
  const ros::Time now = ros::Time::now();
  if (!have_odom_)
  {
    cacheZeroCommand(now);
    publishStatus("WAITING_FOR_ODOM");
    return;
  }
  if (!have_path_ || current_path_.empty())
  {
    cacheZeroCommand(now);
    publishStatus("WAITING_FOR_PATH");
    return;
  }
  if (require_map_ && !have_map_)
  {
    cacheZeroCommand(now);
    publishStatus("WAITING_FOR_MAP");
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
    publishStatus("GOAL_REACHED");
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
    if (!plan_map_transform_ok && require_map_)
    {
      cacheZeroCommand(now);
      publishLocalPlan(TrajectoryCandidate{});
      publishStatus("MAP_TF_ERROR");
      return;
    }
  }

  OccupancyQueryConfig occupancy_config;
  occupancy_config.robot_radius_m = planner_config_.robot_radius_m;
  occupancy_config.clearance_radius_m = planner_config_.clearance_radius_m;
  occupancy_config.lethal_occupancy = planner_config_.lethal_occupancy;
  occupancy_config.treat_unknown_as_occupied = planner_config_.treat_unknown_as_occupied;
  OccupancyAdapter occupancy_adapter(map && plan_map_transform_ok ? map : nullptr, plan_to_map, occupancy_config);

  const double min_progress_s = have_last_progress_ ?
                                  std::max(0.0, last_progress_s_ - planner_config_.progress_rollback_tolerance_m) :
                                  0.0;
  const double max_progress_s = have_last_progress_ ?
                                  last_progress_s_ + planner_config_.max_progress_advance_per_step_m :
                                  planner_config_.lookahead_distance_m;
  PlanResult result = planner_.plan(state, current_path_, occupancy_adapter.hasGrid() ? &occupancy_adapter : nullptr,
                                    min_progress_s, max_progress_s);
  result.diagnostics.plan_map_transform_ok = plan_map_transform_ok;
  if (result.diagnostics.has_initial_match && (result.status == "TRACKING" || result.status == "GOAL_REACHED"))
  {
    last_progress_s_ = have_last_progress_ ? std::max(last_progress_s_, result.diagnostics.initial_progress_s) :
                                             result.diagnostics.initial_progress_s;
    have_last_progress_ = true;
  }
  const ros::Time result_stamp = ros::Time::now();
  publishLocalPlan(result.best);
  publishDiagnostics(state, result);
  if (result.valid && result.status == "TRACKING")
  {
    cacheCommand(result.command, result_stamp, true);
  }
  else
  {
    cacheZeroCommand(result_stamp);
  }

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

void LtDwaAdapterROS::commandPublishTimerCallback(const ros::TimerEvent&)
{
  const Command command = commandForPublish(ros::Time::now());
  publishShadowCommand(command);
  publishCommand(command);
}

void LtDwaAdapterROS::publishCommand(const Command& command)
{
  if (!publish_cmd_vel_)
    return;
  const Command clamped = clampCommand(command);
  geometry_msgs::Twist msg;
  msg.linear.x = clamped.v;
  msg.angular.z = clamped.omega;
  cmd_pub_.publish(msg);
}

void LtDwaAdapterROS::publishZeroCommand()
{
  if (!publish_cmd_vel_)
    return;
  cmd_pub_.publish(geometry_msgs::Twist{});
}

void LtDwaAdapterROS::publishShadowCommand(const Command& command)
{
  if (!publish_shadow_cmd_)
    return;
  const Command clamped = clampCommand(command);
  geometry_msgs::Twist msg;
  msg.linear.x = clamped.v;
  msg.angular.z = clamped.omega;
  shadow_cmd_pub_.publish(msg);
}

void LtDwaAdapterROS::publishStatus(const std::string& status)
{
  if (status == last_status_)
    return;
  last_status_ = status;
  std_msgs::String msg;
  msg.data = status;
  status_pub_.publish(msg);
}

void LtDwaAdapterROS::publishDiagnostics(const RobotState& state, const PlanResult& result)
{
  if (!publish_diagnostics_)
    return;

  const std::string plan_frame = currentPlanFrame();
  const std::string map_frame = have_map_ && !latest_map_.header.frame_id.empty() ? latest_map_.header.frame_id : std::string("");
  const auto& diagnostics = result.diagnostics;
  const auto& collision = diagnostics.initial_collision_details;

  const auto transform_diagnostic_pose = [this](const geometry_msgs::PoseStamped& input,
                                                const std::string& target_frame,
                                                geometry_msgs::PoseStamped& output) -> bool {
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
      output = tf_buffer_.transform(stamped, target_frame, ros::Duration(0.0));
      output.header.stamp = ros::Time(0);
      return true;
    }
    catch (const tf2::TransformException&)
    {
      return false;
    }
  };

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << "status=" << result.status;
  ss << " plan_frame=" << plan_frame;
  ss << " map_frame=" << (map_frame.empty() ? std::string("<none>") : map_frame);
  ss << " plan_map_transform_ok=" << (diagnostics.plan_map_transform_ok ? 1 : 0);
  ss << " path_size=" << current_path_.size();
  ss << " tracker_progress_s=" << (have_last_progress_ ? last_progress_s_ : 0.0);
  ss << " state_plan_x=" << state.x << " state_plan_y=" << state.y << " state_plan_yaw=" << state.yaw;
  ss << " state_v=" << state.v << " state_w=" << state.omega;

  const auto& raw_position = latest_odom_.pose.pose.position;
  ss << " raw_odom_frame=" << (latest_odom_.header.frame_id.empty() ? std::string("<empty>") : latest_odom_.header.frame_id);
  ss << " raw_odom_child=" << (latest_odom_.child_frame_id.empty() ? std::string("<empty>") : latest_odom_.child_frame_id);
  ss << " raw_odom_x=" << raw_position.x << " raw_odom_y=" << raw_position.y;
  ss << " raw_odom_yaw=" << tf2::getYaw(latest_odom_.pose.pose.orientation);

  geometry_msgs::PoseStamped state_plan_pose;
  state_plan_pose.header.frame_id = plan_frame;
  state_plan_pose.header.stamp = ros::Time(0);
  state_plan_pose.pose.position.x = state.x;
  state_plan_pose.pose.position.y = state.y;
  state_plan_pose.pose.orientation = yawToQuat(state.yaw);

  if (!map_frame.empty())
  {
    geometry_msgs::PoseStamped state_map_pose;
    if (transform_diagnostic_pose(state_plan_pose, map_frame, state_map_pose))
    {
      ss << " state_map_x=" << state_map_pose.pose.position.x;
      ss << " state_map_y=" << state_map_pose.pose.position.y;
      ss << " state_map_yaw=" << tf2::getYaw(state_map_pose.pose.orientation);
    }

    geometry_msgs::PoseStamped map_origin;
    map_origin.header.frame_id = map_frame;
    map_origin.header.stamp = ros::Time(0);
    map_origin.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped map_origin_plan;
    if (transform_diagnostic_pose(map_origin, plan_frame, map_origin_plan))
    {
      ss << " map_origin_in_plan_x=" << map_origin_plan.pose.position.x;
      ss << " map_origin_in_plan_y=" << map_origin_plan.pose.position.y;
      ss << " map_origin_in_plan_yaw=" << tf2::getYaw(map_origin_plan.pose.orientation);
    }
  }

  if (diagnostics.has_initial_match)
  {
    ss << " match_idx=" << diagnostics.initial_match_index;
    ss << " match_progress_s=" << diagnostics.initial_progress_s;
    ss << " match_dist=" << diagnostics.initial_match_distance;
    ss << " signed_lateral_err=" << diagnostics.initial_signed_lateral_error;
    ss << " match_heading_err=" << diagnostics.initial_match_heading_error;
  }
  ss << " target_idx=" << diagnostics.lookahead_target_index;
  ss << " target_progress_s=" << diagnostics.lookahead_target_progress_s;
  ss << " target_x=" << diagnostics.lookahead_target_x;
  ss << " target_y=" << diagnostics.lookahead_target_y;
  ss << " tracking_diverged=" << (diagnostics.tracking_diverged ? 1 : 0);
  ss << " max_tracking_deviation_m=" << diagnostics.max_tracking_deviation_m;
  ss << " initial_collision=" << (diagnostics.initial_collision ? 1 : 0);
  ss << " collision_samples=" << collision.checked_samples;
  ss << " collision_unknown=" << collision.unknown_samples;
  ss << " collision_out_of_map=" << collision.out_of_map_samples;
  ss << " collision_lethal=" << collision.lethal_samples;
  ss << " collision_center_occ=" << collision.center_occupancy;
  ss << " collision_max_occ=" << collision.max_occupancy;
  if (collision.has_first_lethal_sample)
  {
    ss << " first_lethal_x=" << collision.first_lethal_x;
    ss << " first_lethal_y=" << collision.first_lethal_y;
  }

  std_msgs::String msg;
  msg.data = ss.str();
  diagnostics_pub_.publish(msg);
}

void LtDwaAdapterROS::publishGlobalPlan()
{
  if (!have_path_ && current_path_msg_.poses.empty())
    return;
  global_plan_pub_.publish(current_path_msg_);
}

void LtDwaAdapterROS::publishLocalPlan(const TrajectoryCandidate& trajectory)
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
}  // namespace lt_dwa_adapter
