#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

geometry_msgs::Quaternion yawToQuat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}
}  // namespace

class BaselineLocalPlannerRunner
{
public:
  BaselineLocalPlannerRunner()
    : private_nh_("~")
    , loader_("nav_core", "nav_core::BaseLocalPlanner")
    , tf_listener_(tf_buffer_)
  {
    private_nh_.param("plugin_type", plugin_type_, std::string("teb_local_planner/TebLocalPlannerROS"));
    private_nh_.param("plugin_name", plugin_name_, std::string("TebLocalPlannerROS"));
    private_nh_.param("global_path_topic", global_path_topic_, std::string("/scout/global_path_fixed"));
    private_nh_.param("goal_topic", goal_topic_, std::string("/scout/goal"));
    private_nh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    private_nh_.param("status_topic", status_topic_, std::string("/baseline/status"));
    private_nh_.param("global_plan_topic", global_plan_topic_, std::string("/baseline/global_plan"));
    private_nh_.param("controller_frequency", controller_frequency_, 10.0);
    private_nh_.param("goal_replan_on_receive", goal_replan_on_receive_, true);
    private_nh_.param("force_straight_plan_on_goal", force_straight_plan_on_goal_, false);
    private_nh_.param("use_wrapper_goal_check", use_wrapper_goal_check_, true);
    private_nh_.param("latch_goal_reached", latch_goal_reached_, false);
    private_nh_.param("straight_plan_spacing", straight_plan_spacing_, 0.05);
    private_nh_.param("xy_goal_tolerance", xy_goal_tolerance_, 0.20);
    private_nh_.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.20);
    private_nh_.param("max_cmd_vel_x", max_cmd_vel_x_, 0.8);
    private_nh_.param("max_cmd_vel_theta", max_cmd_vel_theta_, 1.2);
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));
    private_nh_.param("plan_target_frame", plan_target_frame_, std::string(""));

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
    global_plan_pub_ = nh_.advertise<nav_msgs::Path>(global_plan_topic_, 1, true);

    path_sub_ = nh_.subscribe(global_path_topic_, 1, &BaselineLocalPlannerRunner::pathCallback, this);
    goal_sub_ = nh_.subscribe(goal_topic_, 1, &BaselineLocalPlannerRunner::goalCallback, this);

    costmap_ros_.reset(new costmap_2d::Costmap2DROS("local_costmap", tf_buffer_));
    costmap_ros_->pause();
    costmap_ros_->start();

    planner_ = loader_.createInstance(plugin_type_);
    planner_->initialize(plugin_name_, &tf_buffer_, costmap_ros_.get());

    ROS_INFO_STREAM("[baseline_runner] loaded plugin_type=" << plugin_type_ << " plugin_name=" << plugin_name_);
  }

  void spin()
  {
    ros::Rate rate(controller_frequency_);
    while (ros::ok())
    {
      ros::spinOnce();
      step();
      rate.sleep();
    }
  }

private:
  void pathCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    if (msg->poses.empty())
      return;
    current_plan_ = msg->poses;
    plan_frame_ = msg->header.frame_id.empty() ? msg->poses.front().header.frame_id : msg->header.frame_id;
    for (auto& pose : current_plan_)
    {
      if (pose.header.frame_id.empty())
        pose.header.frame_id = plan_frame_;
      pose.header.stamp = ros::Time(0);
    }
    if (!plan_target_frame_.empty() && plan_frame_ != plan_target_frame_)
    {
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      transformed_plan.reserve(current_plan_.size());
      try
      {
        for (const auto& pose : current_plan_)
        {
          geometry_msgs::PoseStamped transformed = tf_buffer_.transform(pose, plan_target_frame_, ros::Duration(0.2));
          transformed.header.stamp = ros::Time(0);
          transformed_plan.push_back(transformed);
        }
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(2.0, "[baseline_runner] transform plan %s -> %s failed: %s",
                          plan_frame_.c_str(), plan_target_frame_.c_str(), ex.what());
        return;
      }
      current_plan_.swap(transformed_plan);
      plan_frame_ = plan_target_frame_;
    }
    if (!have_goal_)
    {
      goal_ = current_plan_.back();
      have_goal_ = true;
    }
    goal_reached_latched_ = false;
    plan_dirty_ = true;
    publishStatus("PATH_RECEIVED");
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    goal_ = *msg;
    if (!plan_target_frame_.empty() && goal_.header.frame_id != plan_target_frame_)
    {
      if (goal_.header.frame_id.empty())
        goal_.header.frame_id = "map";
      goal_.header.stamp = ros::Time(0);
      try
      {
        goal_ = tf_buffer_.transform(goal_, plan_target_frame_, ros::Duration(0.2));
        goal_.header.stamp = ros::Time(0);
      }
      catch (const tf2::TransformException& ex)
      {
        ROS_WARN_THROTTLE(2.0, "[baseline_runner] transform goal %s -> %s failed: %s",
                          msg->header.frame_id.c_str(), plan_target_frame_.c_str(), ex.what());
        return;
      }
    }
    have_goal_ = true;
    goal_reached_latched_ = false;
    if (force_straight_plan_on_goal_ || (goal_replan_on_receive_ && current_plan_.empty()))
      buildStraightPlanToGoal(goal_);
    publishStatus("GOAL_RECEIVED");
  }

  bool getRobotPoseInFrame(const std::string& frame, geometry_msgs::PoseStamped& pose) const
  {
    geometry_msgs::PoseStamped base_pose;
    base_pose.header.stamp = ros::Time(0);
    base_pose.header.frame_id = base_frame_;
    base_pose.pose.orientation.w = 1.0;
    try
    {
      pose = tf_buffer_.transform(base_pose, frame, ros::Duration(0.2));
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(2.0, "[baseline_runner] TF %s -> %s failed: %s", base_frame_.c_str(), frame.c_str(), ex.what());
      return false;
    }
  }

  void buildStraightPlanToGoal(const geometry_msgs::PoseStamped& goal)
  {
    const std::string frame = goal.header.frame_id.empty() ? std::string("map") : goal.header.frame_id;
    geometry_msgs::PoseStamped start;
    if (!getRobotPoseInFrame(frame, start))
      return;

    const double sx = start.pose.position.x;
    const double sy = start.pose.position.y;
    const double gx = goal.pose.position.x;
    const double gy = goal.pose.position.y;
    const double dx = gx - sx;
    const double dy = gy - sy;
    const double dist = std::hypot(dx, dy);
    const int steps = std::max(2, static_cast<int>(std::ceil(dist / std::max(0.01, straight_plan_spacing_))));
    const double heading = std::atan2(dy, dx);
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);

    current_plan_.clear();
    current_plan_.reserve(static_cast<size_t>(steps + 1));
    for (int i = 0; i <= steps; ++i)
    {
      const double r = static_cast<double>(i) / static_cast<double>(steps);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = frame;
      pose.header.stamp = ros::Time(0);
      pose.pose.position.x = sx + r * dx;
      pose.pose.position.y = sy + r * dy;
      pose.pose.orientation = yawToQuat(i == steps ? goal_yaw : heading);
      current_plan_.push_back(pose);
    }
    plan_frame_ = frame;
    plan_dirty_ = true;
  }

  bool goalCloseEnough() const
  {
    if (!have_goal_)
      return false;
    const std::string frame = goal_.header.frame_id.empty() ? std::string("map") : goal_.header.frame_id;
    geometry_msgs::PoseStamped robot;
    if (!getRobotPoseInFrame(frame, robot))
      return false;
    const double dx = goal_.pose.position.x - robot.pose.position.x;
    const double dy = goal_.pose.position.y - robot.pose.position.y;
    const double dist = std::hypot(dx, dy);
    const double yaw_err = std::fabs(normalizeAngle(tf2::getYaw(goal_.pose.orientation) - tf2::getYaw(robot.pose.orientation)));
    return dist <= xy_goal_tolerance_ && yaw_err <= yaw_goal_tolerance_;
  }

  void setPlanIfNeeded()
  {
    if (!plan_dirty_ || current_plan_.empty())
      return;
    if (planner_->setPlan(current_plan_))
    {
      plan_dirty_ = false;
      publishGlobalPlan();
      publishStatus("PLAN_SET");
    }
    else
    {
      publishStatus("SET_PLAN_FAILED");
    }
  }

  void step()
  {
    setPlanIfNeeded();
    if (current_plan_.empty())
    {
      publishZero();
      publishStatus("WAITING_FOR_PLAN");
      return;
    }

    bool goal_reached = latch_goal_reached_ && goal_reached_latched_;
    if (!goal_reached && use_wrapper_goal_check_)
      goal_reached = goalCloseEnough();
    if (!goal_reached && planner_)
      goal_reached = planner_->isGoalReached();

    if (goal_reached)
    {
      if (latch_goal_reached_)
        goal_reached_latched_ = true;
      publishZero();
      publishStatus("GOAL_REACHED");
      return;
    }

    geometry_msgs::Twist cmd;
    if (planner_->computeVelocityCommands(cmd))
    {
      clampCommand(cmd);
      cmd_pub_.publish(cmd);
      publishStatus("TRACKING");
    }
    else
    {
      publishZero();
      publishStatus("NO_VALID_CMD");
    }
  }

  void clampCommand(geometry_msgs::Twist& cmd) const
  {
    if (max_cmd_vel_x_ > 0.0)
    {
      cmd.linear.x = std::max(-max_cmd_vel_x_, std::min(max_cmd_vel_x_, cmd.linear.x));
      cmd.linear.y = std::max(-max_cmd_vel_x_, std::min(max_cmd_vel_x_, cmd.linear.y));
    }
    if (max_cmd_vel_theta_ > 0.0)
      cmd.angular.z = std::max(-max_cmd_vel_theta_, std::min(max_cmd_vel_theta_, cmd.angular.z));
  }

  void publishZero()
  {
    geometry_msgs::Twist zero;
    cmd_pub_.publish(zero);
  }

  void publishStatus(const std::string& status)
  {
    if (status == last_status_)
      return;
    last_status_ = status;
    std_msgs::String msg;
    msg.data = status;
    status_pub_.publish(msg);
  }

  void publishGlobalPlan()
  {
    nav_msgs::Path path;
    path.header.stamp = ros::Time(0);
    path.header.frame_id = plan_frame_.empty() ? current_plan_.front().header.frame_id : plan_frame_;
    path.poses = current_plan_;
    global_plan_pub_.publish(path);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> loader_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  boost::shared_ptr<nav_core::BaseLocalPlanner> planner_;

  ros::Subscriber path_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher status_pub_;
  ros::Publisher global_plan_pub_;

  std::vector<geometry_msgs::PoseStamped> current_plan_;
  geometry_msgs::PoseStamped goal_;
  std::string plan_frame_;
  std::string last_status_;

  std::string plugin_type_;
  std::string plugin_name_;
  std::string global_path_topic_;
  std::string goal_topic_;
  std::string cmd_vel_topic_;
  std::string status_topic_;
  std::string global_plan_topic_;
  std::string base_frame_;
  std::string plan_target_frame_;
  double controller_frequency_ = 10.0;
  double straight_plan_spacing_ = 0.05;
  double xy_goal_tolerance_ = 0.20;
  double yaw_goal_tolerance_ = 0.20;
  double max_cmd_vel_x_ = 0.8;
  double max_cmd_vel_theta_ = 1.2;
  bool goal_replan_on_receive_ = true;
  bool force_straight_plan_on_goal_ = false;
  bool use_wrapper_goal_check_ = true;
  bool latch_goal_reached_ = false;
  bool goal_reached_latched_ = false;
  bool plan_dirty_ = false;
  bool have_goal_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "baseline_local_planner_runner");
  try
  {
    BaselineLocalPlannerRunner runner;
    runner.spin();
  }
  catch (const std::exception& ex)
  {
    ROS_FATAL_STREAM("[baseline_runner] failed: " << ex.what());
    std::cerr << "[baseline_runner] failed: " << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
