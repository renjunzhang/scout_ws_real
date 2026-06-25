#include "lt_dwa_official_wrapper/official_core_runner.hpp"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>

#include <local_map_generation/GetLocalMap.h>
#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/service.h>

#include "lt_dwa_official_wrapper/converters.hpp"
#include "policy/seed_policy.hpp"

namespace lt_dwa_official_wrapper {
namespace {

constexpr const char* kLocalPlannerPackage = "local_planner";
constexpr const char* kPlanningConfig = "/config/planning.config";
constexpr const char* kDataDirectory = "/data";
constexpr const char* kLocalMapService = "/local_map_generation/service";

bool StartsWithPath(const std::string& path, const std::string& prefix) {
  if (prefix.empty() || path.size() < prefix.size()) {
    return false;
  }
  if (path.compare(0, prefix.size(), prefix) != 0) {
    return false;
  }
  return path.size() == prefix.size() || path[prefix.size()] == '/';
}

bool FileExists(const std::string& path) {
  std::ifstream in(path);
  return in.good();
}

bool DirectoryExists(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

OfficialCoreResult MakeResult(WrapperStatus status, const std::string& reason) {
  OfficialCoreResult result;
  result.status = status;
  result.reason = reason;
  return result;
}

void EnsureRosInitialized() {
  if (ros::isInitialized()) {
    return;
  }
  ros::M_string remappings;
  ros::init(remappings,
            "lt_dwa_official_core_worker",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
}

OfficialCoreResult PreflightOfficialRuntime(const std::string& official_source_root,
                                            std::string* local_planner_path) {
  EnsureRosInitialized();

  if (!ros::master::check()) {
    return MakeResult(WrapperStatus::kWaitingForInput, "ros_master_unavailable");
  }

  *local_planner_path = ros::package::getPath(kLocalPlannerPackage);
  if (local_planner_path->empty()) {
    return MakeResult(WrapperStatus::kWaitingForInput, "local_planner_package_not_found");
  }

  if (StartsWithPath(*local_planner_path, official_source_root)) {
    return MakeResult(WrapperStatus::kWaitingForInput,
                      "local_planner_path_points_to_readonly_official_source");
  }

  if (!FileExists(*local_planner_path + kPlanningConfig)) {
    return MakeResult(WrapperStatus::kWaitingForInput, "planning_config_not_found");
  }

  if (!DirectoryExists(*local_planner_path + kDataDirectory)) {
    return MakeResult(WrapperStatus::kWaitingForInput, "runtime_data_directory_not_prepared");
  }

  if (!ros::service::exists(kLocalMapService, false)) {
    return MakeResult(WrapperStatus::kWaitingForInput, "local_map_service_unavailable");
  }

  return MakeResult(WrapperStatus::kOk, "preflight_ok");
}

bool IsGoalReached(const PlannerInput& input, const PlannerConfig& config) {
  const double dx = input.target_pose.x - input.robot_pose.x;
  const double dy = input.target_pose.y - input.robot_pose.y;
  return std::hypot(dx, dy) <= config.robot_radius;
}

constexpr double kPi = 3.14159265358979323846;

double NormalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

double PointDistance(const Pose2d& a, const Pose2d& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

std::size_t FindNearestPathIndex(const PlannerInput& input) {
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < input.reference_path.size(); ++i) {
    const double distance = PointDistance(input.robot_pose, input.reference_path[i]);
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }
  return nearest;
}

std::size_t FindLookaheadPathIndex(const PlannerInput& input,
                                   std::size_t nearest,
                                   double lookahead_m) {
  std::size_t target = nearest;
  double accumulated = 0.0;
  for (std::size_t i = nearest; i + 1 < input.reference_path.size(); ++i) {
    accumulated += PointDistance(input.reference_path[i], input.reference_path[i + 1]);
    target = i + 1;
    if (accumulated >= lookahead_m) {
      break;
    }
  }
  return target;
}

Twist2d PathTrackingGuardCommand(const PlannerInput& input, const PlannerConfig& config) {
  const std::size_t nearest = FindNearestPathIndex(input);
  const double lookahead_m = std::max(0.25, config.path_tracking_lookahead_m);
  const std::size_t target_index = FindLookaheadPathIndex(input, nearest, lookahead_m);
  const Pose2d& target = input.reference_path[target_index];
  const Pose2d& nearest_pose = input.reference_path[nearest];

  const double goal_dist = std::hypot(input.target_pose.x - input.robot_pose.x,
                                      input.target_pose.y - input.robot_pose.y);
  const double dx = target.x - input.robot_pose.x;
  const double dy = target.y - input.robot_pose.y;
  const double cos_yaw = std::cos(input.robot_pose.yaw);
  const double sin_yaw = std::sin(input.robot_pose.yaw);
  const double x_local = cos_yaw * dx + sin_yaw * dy;
  const double y_local = -sin_yaw * dx + cos_yaw * dy;
  const double lookahead_dist = std::max(0.25, std::hypot(x_local, y_local));
  const double alpha = std::atan2(y_local, std::max(0.05, x_local));
  const double heading_error = NormalizeAngle(target.yaw - input.robot_pose.yaw);
  const double nearest_error = PointDistance(input.robot_pose, nearest_pose);

  const double min_v = Clamp(config.path_tracking_min_v, config.min_v, config.max_v);
  double speed_scale = 1.0 / (1.0 + 1.8 * std::abs(alpha) + 0.8 * nearest_error +
                              0.3 * std::abs(heading_error));
  double v = Clamp(config.max_v * speed_scale, min_v, config.max_v);
  if (std::abs(alpha) > 1.2 || nearest_error > 0.6) {
    v = std::min(v, std::max(min_v, 0.35));
  }
  if (goal_dist < 1.2) {
    v = std::min(v, std::max(min_v, 0.65 * goal_dist));
  }

  double w = 2.0 * v * y_local / (lookahead_dist * lookahead_dist) + 0.7 * heading_error;
  w = Clamp(w, -config.max_w, config.max_w);

  const double dv_step = std::max(0.0, config.max_acc * config.time_step);
  const double dw_step = std::max(0.0, config.max_angular_acc * config.time_step);
  v = Clamp(v, input.robot_twist.v - dv_step, input.robot_twist.v + dv_step);
  w = Clamp(w, input.robot_twist.w - dw_step, input.robot_twist.w + dw_step);

  Twist2d command;
  command.v = Clamp(v, config.min_v, config.max_v);
  command.w = Clamp(w, -config.max_w, config.max_w);
  return command;
}

}  // namespace

OfficialCoreResult RunOfficialCoreOnce(const PlannerInput& input,
                                       const PlannerConfig& config,
                                       const std::string& official_source_root) {
  std::string local_planner_path;
  OfficialCoreResult preflight = PreflightOfficialRuntime(official_source_root, &local_planner_path);
  if (preflight.status != WrapperStatus::kOk) {
    return preflight;
  }

  try {
    ros::NodeHandle nh;
    ros::ServiceClient local_map_service =
        nh.serviceClient<local_map_generation::GetLocalMap>(kLocalMapService, true);
    if (!local_map_service.waitForExistence(ros::Duration(0.5))) {
      return MakeResult(WrapperStatus::kWaitingForInput, "local_map_service_wait_failed");
    }

    Robot robot(config.max_v,
                config.min_v,
                config.max_w,
                config.max_acc,
                config.max_angular_acc,
                ToOfficialPose(input.robot_pose),
                ToOfficialAction(input.robot_twist),
                false);

    SeedPolicy policy(config.max_v,
                      config.min_v,
                      config.max_w,
                      config.max_acc,
                      config.max_angular_acc,
                      config.robot_radius,
                      config.time_step,
                      local_map_service);

    Action planned_action;
    const int core_return = policy.forward(robot,
                                           ToOfficialPose(input.target_pose),
                                           ToOfficialPath(input.reference_path, config.path_resample_spacing),
                                           ToOfficialGridMap(input.occupancy_grid),
                                           ToOfficialObstacleHistory(input.dynamic_obstacles),
                                           planned_action);

    OfficialCoreResult result;
    result.core_return = core_return;
    result.raw_command.v = planned_action.v_;
    result.raw_command.w = planned_action.w_;
    result.final_command = result.raw_command;
    result.guard_reason = "pass_through_official_core";

    if (core_return < 0) {
      result.status = WrapperStatus::kCorePlanningFailed;
      result.reason = "official_core_returned_failure";
      result.raw_command.v = 0.0;
      result.raw_command.w = 0.0;
      result.final_command.v = 0.0;
      result.final_command.w = 0.0;
      result.guard_reason = "core_planning_failed";
      return result;
    }

    if (IsGoalReached(input, config)) {
      result.status = WrapperStatus::kGoalReached;
      result.reason = "official_core_goal_reached";
      result.raw_command.v = 0.0;
      result.raw_command.w = 0.0;
      result.final_command.v = 0.0;
      result.final_command.w = 0.0;
      result.guard_reason = "goal_reached";
      return result;
    }

    if (config.enable_path_tracking_guard && input.reference_path.size() >= 2) {
      result.final_command = PathTrackingGuardCommand(input, config);
      result.guard_applied = true;
      result.guard_reason = "path_tracking_guard";
      result.reason = "official_core_ok_path_tracking_guard";
    } else {
      result.reason = "official_core_ok";
    }
    result.status = WrapperStatus::kOk;
    return result;
  } catch (const std::exception& ex) {
    return MakeResult(WrapperStatus::kCoreException,
                      std::string("official_core_exception_") + ex.what());
  } catch (...) {
    return MakeResult(WrapperStatus::kCoreException, "official_core_unknown_exception");
  }
}

}  // namespace lt_dwa_official_wrapper
