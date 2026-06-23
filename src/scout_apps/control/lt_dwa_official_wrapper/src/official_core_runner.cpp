#include "lt_dwa_official_wrapper/official_core_runner.hpp"

#include <sys/stat.h>

#include <cmath>
#include <fstream>
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
                ToOfficialAction(input.robot_twist));

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
    result.command.v = planned_action.v_;
    result.command.w = planned_action.w_;

    if (core_return < 0) {
      result.status = WrapperStatus::kCorePlanningFailed;
      result.reason = "official_core_returned_failure";
      result.command.v = 0.0;
      result.command.w = 0.0;
      return result;
    }

    if (IsGoalReached(input, config)) {
      result.status = WrapperStatus::kGoalReached;
      result.reason = "official_core_goal_reached";
      result.command.v = 0.0;
      result.command.w = 0.0;
      return result;
    }

    result.status = WrapperStatus::kOk;
    result.reason = "official_core_ok";
    return result;
  } catch (const std::exception& ex) {
    return MakeResult(WrapperStatus::kCoreException,
                      std::string("official_core_exception_") + ex.what());
  } catch (...) {
    return MakeResult(WrapperStatus::kCoreException, "official_core_unknown_exception");
  }
}

}  // namespace lt_dwa_official_wrapper
