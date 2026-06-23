#pragma once

#include <string>
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/time.h>

#include "lt_dwa_official_wrapper/status.hpp"

namespace lt_dwa_official_wrapper {

struct Pose2d {
  std::string frame_id;
  ros::Time stamp;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Twist2d {
  double v{0.0};
  double w{0.0};
};

struct ObstacleTrack {
  int id{0};
  std::string frame_id;
  ros::Time stamp;
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double radius{0.0};
};

struct PlannerInput {
  std::string planning_frame;
  ros::Time stamp;
  Pose2d robot_pose;
  Twist2d robot_twist;
  Pose2d target_pose;
  std::vector<Pose2d> reference_path;
  nav_msgs::OccupancyGrid occupancy_grid;
  std::vector<ObstacleTrack> dynamic_obstacles;
};

struct PlannerDiagnostics {
  std::string status;
  std::string reject_reason;
  std::string planning_frame;
  double input_stamp_age_sec{0.0};
  std::size_t path_points_raw{0};
  std::size_t path_points_resampled{0};
  double path_length_m{0.0};
  double goal_dist_m{0.0};
  double goal_yaw_err_rad{0.0};
  unsigned int map_width{0};
  unsigned int map_height{0};
  double map_resolution{0.0};
  std::size_t obstacle_count{0};
  double planner_latency_ms{0.0};
  double command_raw_v{0.0};
  double command_raw_w{0.0};
  bool command_rejected{false};
  unsigned int deterministic_seed{0};
};

struct PlannerOutput {
  WrapperStatus status{WrapperStatus::kWaitingForInput};
  Twist2d command_raw;
  PlannerDiagnostics diagnostics;
};

}  // namespace lt_dwa_official_wrapper
