#include "lt_dwa_official_wrapper/planner_facade.hpp"

#include <cstdlib>
#include <cmath>
#include <string>

#include "lt_dwa_official_wrapper/converters.hpp"

namespace lt_dwa_official_wrapper {
namespace {

double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace

PlannerFacade::PlannerFacade(const PlannerConfig& config) : config_(config) {
  std::srand(config_.deterministic_seed);
}

PlannerOutput PlannerFacade::PlanOnce(const PlannerInput& input) const {
  return PlanOnce(input, ros::Time());
}

PlannerOutput PlannerFacade::PlanOnce(const PlannerInput& input, const ros::Time& now) const {
  const auto validation = validator_.ValidateInput(input, config_, now);
  if (!validation.ok()) {
    return MakeRejectedOutput(input, validation.status, validation.reason, now);
  }

  auto diagnostics = BuildBaseDiagnostics(input, now);

  const auto official_target = ToOfficialPose(input.target_pose);
  const auto official_current_action = ToOfficialAction(input.robot_twist);
  const auto official_path = ToOfficialPath(input.reference_path, config_.path_resample_spacing);
  const auto official_map = ToOfficialGridMap(input.occupancy_grid);
  const auto official_obstacles = ToOfficialObstacleHistory(input.dynamic_obstacles);

  diagnostics.path_points_resampled = official_path.size();
  diagnostics.obstacle_count = official_obstacles.size();
  diagnostics.map_width = static_cast<unsigned int>(official_map.getWidth());
  diagnostics.map_height = static_cast<unsigned int>(official_map.getHeight());
  diagnostics.map_resolution = official_map.getResolution();

  if (official_path.empty()) {
    diagnostics.status = ToString(WrapperStatus::kDegeneratePath);
    diagnostics.reject_reason = "converted official path is empty";
    diagnostics.command_rejected = true;
    PlannerOutput output;
    output.status = WrapperStatus::kDegeneratePath;
    output.diagnostics = diagnostics;
    return output;
  }

  // Phase 3C intentionally stops before calling official SeedPolicy::forward(...).
  // The variables above prove the wrapper can validate and convert inputs to the
  // official data model, but no command is produced until Phase 3D/next approval.
  (void)official_target;
  (void)official_current_action;

  diagnostics.status = ToString(WrapperStatus::kCommandRejected);
  diagnostics.reject_reason = "official core call disabled in Phase 3C PlannerFacade skeleton";
  diagnostics.command_rejected = true;
  diagnostics.command_raw_v = 0.0;
  diagnostics.command_raw_w = 0.0;

  PlannerOutput output;
  output.status = WrapperStatus::kCommandRejected;
  output.command_raw.v = 0.0;
  output.command_raw.w = 0.0;
  output.diagnostics = diagnostics;
  return output;
}

PlannerOutput PlannerFacade::MakeRejectedOutput(const PlannerInput& input,
                                                WrapperStatus status,
                                                const std::string& reason,
                                                const ros::Time& now) const {
  PlannerOutput output;
  output.status = status;
  output.command_raw.v = 0.0;
  output.command_raw.w = 0.0;
  output.diagnostics = BuildBaseDiagnostics(input, now);
  output.diagnostics.status = ToString(status);
  output.diagnostics.reject_reason = reason;
  output.diagnostics.command_rejected = true;
  output.diagnostics.command_raw_v = 0.0;
  output.diagnostics.command_raw_w = 0.0;
  return output;
}

PlannerDiagnostics PlannerFacade::BuildBaseDiagnostics(const PlannerInput& input,
                                                       const ros::Time& now) const {
  PlannerDiagnostics diagnostics;
  diagnostics.status = ToString(WrapperStatus::kWaitingForInput);
  diagnostics.planning_frame = config_.planning_frame;
  diagnostics.path_points_raw = input.reference_path.size();
  diagnostics.path_length_m = ComputePathLength(input.reference_path);
  diagnostics.map_width = input.occupancy_grid.info.width;
  diagnostics.map_height = input.occupancy_grid.info.height;
  diagnostics.map_resolution = input.occupancy_grid.info.resolution;
  diagnostics.obstacle_count = input.dynamic_obstacles.size();
  diagnostics.deterministic_seed = config_.deterministic_seed;

  if (!now.isZero() && !input.stamp.isZero()) {
    diagnostics.input_stamp_age_sec = (now - input.stamp).toSec();
  }

  diagnostics.goal_dist_m = std::hypot(input.target_pose.x - input.robot_pose.x,
                                       input.target_pose.y - input.robot_pose.y);
  diagnostics.goal_yaw_err_rad = NormalizeAngle(input.target_pose.yaw - input.robot_pose.yaw);
  return diagnostics;
}

}  // namespace lt_dwa_official_wrapper
