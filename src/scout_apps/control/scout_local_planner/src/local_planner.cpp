#include "scout_local_planner/local_planner.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>

namespace scout_local_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_) {
}

bool LocalPlanner::initialize() {
  // 加载参数
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");
  pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<double>("goal_tolerance", goal_tolerance_, 0.1);
  pnh_.param<double>("lookahead_distance", lookahead_distance_, 1.0);
  pnh_.param<double>("control_rate", control_rate_, 20.0);

  // 液体晃动模型参数
  slosh_models::LiquidSloshModel::Params slosh_params;
  pnh_.param<double>("slosh/container_radius", slosh_params.R, 0.15);
  pnh_.param<double>("slosh/liquid_height", slosh_params.h, 0.20);
  pnh_.param<double>("slosh/liquid_density", slosh_params.rho, 1000.0);
  pnh_.param<int>("slosh/mode_index", slosh_params.mode_index, 1);
  pnh_.param<double>("slosh/damping_ratio", slosh_params.zeta, 0.05);
  pnh_.param<double>("slosh/dt", slosh_params.dt, 0.05);
  pnh_.param<bool>("slosh/use_linear_model", slosh_params.use_linear_model, true);
  pnh_.param<bool>("slosh/use_parabola_term", slosh_params.use_parabola_term, true);

  // MPC参数
  slosh_models::MPCVelTracker::Params mpc_params;
  pnh_.param<double>("mpc/dt", mpc_params.dt, 0.05);
  pnh_.param<int>("mpc/horizon", mpc_params.horizon, 15);
  pnh_.param<double>("mpc/max_speed", mpc_params.max_speed, 0.5);
  pnh_.param<double>("mpc/min_speed", mpc_params.min_speed, 0.0);
  pnh_.param<bool>("mpc/enable_slosh", mpc_params.enable_slosh, true);
  pnh_.param<double>("mpc/q_slosh", mpc_params.q_slosh, 10.0);
  pnh_.param<double>("mpc/max_slosh_height", mpc_params.max_slosh_height, 0.05);

  double q_pos, r_vel, s_delta;
  pnh_.param<double>("mpc/q_pos", q_pos, 10.0);
  pnh_.param<double>("mpc/r_vel", r_vel, 0.2);
  pnh_.param<double>("mpc/s_delta", s_delta, 0.05);
  mpc_params.q_pos = Eigen::Vector2d(q_pos, q_pos);
  mpc_params.r_vel = Eigen::Vector2d(r_vel, r_vel);
  mpc_params.s_delta = Eigen::Vector2d(s_delta, s_delta);

  // 初始化液体晃动模型
  bool slosh_enabled = false;
  pnh_.param<bool>("slosh/enabled", slosh_enabled, true);

  if (slosh_enabled && mpc_params.enable_slosh) {
    slosh_model_ = std::make_shared<slosh_models::LiquidSloshModel>();
    if (!slosh_model_->configure(slosh_params)) {
      ROS_ERROR("[LocalPlanner] Failed to configure slosh model");
      return false;
    }
  }

  // 初始化MPC控制器
  mpc_tracker_ = std::make_unique<slosh_models::MPCVelTracker>();
  if (!mpc_tracker_->configure(mpc_params)) {
    ROS_ERROR("[LocalPlanner] Failed to configure MPC tracker");
    return false;
  }

  // 绑定液体晃动模型
  if (slosh_model_) {
    if (!mpc_tracker_->setSloshModel(slosh_model_)) {
      ROS_ERROR("[LocalPlanner] Failed to set slosh model to MPC tracker");
      return false;
    }
  }

  // 设置订阅和发布
  odom_sub_ = nh_.subscribe("odom", 1, &LocalPlanner::odomCallback, this);
  path_sub_ = nh_.subscribe("global_path", 1, &LocalPlanner::pathCallback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  local_path_pub_ = nh_.advertise<nav_msgs::Path>("local_path", 1);
  slosh_height_pub_ = nh_.advertise<std_msgs::Float64>("slosh_height", 1);

  ROS_INFO("[LocalPlanner] 初始化完成");
  return true;
}

void LocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_odom_ = *msg;
  has_odom_ = true;
}

void LocalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
  global_path_ = *msg;
  has_path_ = !global_path_.poses.empty();
  current_waypoint_idx_ = 0;
  ROS_INFO("[LocalPlanner] 收到全局路径，共 %zu 个路点", global_path_.poses.size());
}

void LocalPlanner::setGlobalPath(const nav_msgs::Path& path) {
  global_path_ = path;
  has_path_ = !global_path_.poses.empty();
  current_waypoint_idx_ = 0;
}

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!has_odom_ || !has_path_) {
    ROS_WARN_THROTTLE(1.0, "[LocalPlanner] 等待里程计或路径数据");
    return false;
  }

  // 获取当前位置
  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose)) {
    return false;
  }

  // 查找最近路点
  int closest_idx = findClosestWaypoint();
  if (closest_idx < 0) {
    return false;
  }

  // 检查是否到达目标
  if (isGoalReached()) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  // 生成参考速度序列
  std::vector<Eigen::Vector2d> ref_vel = generateReferenceVelocities(mpc_tracker_->params().horizon);

  // 计算位置误差
  const auto& target_pose = global_path_.poses[closest_idx].pose;
  Eigen::Vector2d pos_error(
    target_pose.position.x - robot_pose.pose.position.x,
    target_pose.position.y - robot_pose.pose.position.y
  );

  // 获取当前角速度
  double omega_z = current_odom_.twist.twist.angular.z;
  double alpha_z = 0.0;  // 简化：不考虑角加速度

  // MPC求解
  slosh_models::MPCResult result = mpc_tracker_->compute(pos_error, ref_vel, omega_z, alpha_z);

  if (!result.success) {
    ROS_WARN_THROTTLE(1.0, "[LocalPlanner] MPC求解失败");
    return false;
  }

  // 设置速度指令
  cmd_vel.linear.x = result.command.x();
  cmd_vel.linear.y = result.command.y();
  cmd_vel.angular.z = 0.0;  // TODO: 添加航向控制

  // 发布晃动高度
  if (slosh_model_) {
    std_msgs::Float64 slosh_msg;
    slosh_msg.data = result.current_slosh_height;
    slosh_height_pub_.publish(slosh_msg);
  }

  return true;
}

bool LocalPlanner::isGoalReached() const {
  if (!has_path_ || global_path_.poses.empty()) {
    return false;
  }

  const auto& goal = global_path_.poses.back().pose.position;
  double dx = goal.x - current_odom_.pose.pose.position.x;
  double dy = goal.y - current_odom_.pose.pose.position.y;
  double dist = std::sqrt(dx*dx + dy*dy);

  return dist < goal_tolerance_;
}

double LocalPlanner::getCurrentSloshHeight() const {
  if (slosh_model_) {
    return slosh_model_->getSloshHeight();
  }
  return 0.0;
}

bool LocalPlanner::getRobotPose(geometry_msgs::PoseStamped& pose) {
  try {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
      map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));

    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "[LocalPlanner] TF异常: %s", ex.what());
    return false;
  }
}

int LocalPlanner::findClosestWaypoint() {
  if (global_path_.poses.empty()) {
    return -1;
  }

  double min_dist = std::numeric_limits<double>::max();
  int closest_idx = current_waypoint_idx_;

  for (size_t i = current_waypoint_idx_; i < global_path_.poses.size(); ++i) {
    const auto& wp = global_path_.poses[i].pose.position;
    double dx = wp.x - current_odom_.pose.pose.position.x;
    double dy = wp.y - current_odom_.pose.pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }

    // 如果距离开始增大，停止搜索
    if (dist > min_dist + 0.5) {
      break;
    }
  }

  current_waypoint_idx_ = closest_idx;
  return closest_idx;
}

std::vector<Eigen::Vector2d> LocalPlanner::generateReferenceVelocities(int horizon) {
  std::vector<Eigen::Vector2d> ref_vel(horizon, Eigen::Vector2d::Zero());

  if (global_path_.poses.empty() || current_waypoint_idx_ >= static_cast<int>(global_path_.poses.size()) - 1) {
    return ref_vel;
  }

  double dt = mpc_tracker_->params().dt;
  double max_speed = mpc_tracker_->params().max_speed;

  for (int i = 0; i < horizon; ++i) {
    int wp_idx = std::min(current_waypoint_idx_ + i + 1, static_cast<int>(global_path_.poses.size()) - 1);
    int prev_idx = std::max(wp_idx - 1, 0);

    const auto& wp = global_path_.poses[wp_idx].pose.position;
    const auto& prev_wp = global_path_.poses[prev_idx].pose.position;

    double dx = wp.x - prev_wp.x;
    double dy = wp.y - prev_wp.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist > 1e-6) {
      double speed = std::min(dist / dt, max_speed);
      ref_vel[i] = Eigen::Vector2d(speed * dx / dist, speed * dy / dist);
    }
  }

  return ref_vel;
}

}  // namespace scout_local_planner
