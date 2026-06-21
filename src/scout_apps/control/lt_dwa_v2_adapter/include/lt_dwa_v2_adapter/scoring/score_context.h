#pragma once

#include "lt_dwa_v2_adapter/core/planner_config.h"
#include "lt_dwa_v2_adapter/core/trajectory_types.h"
#include "lt_dwa_v2_adapter/geometry/path_reference.h"
#include "lt_dwa_v2_adapter/world/occupancy_adapter.h"

namespace lt_dwa_v2_adapter
{
struct ScoreContext
{
  const PlannerConfig* config = nullptr;
  const PathReference* path = nullptr;
  const OccupancyAdapter* occupancy = nullptr;

  RobotState state;
  Command command;
  Command previous_command;
  double previous_progress_s = 0.0;

  PathProjection match;
  Pose2D goal;
  Pose2D target;

  double terminal_dist = 0.0;
  double path_scale = 1.0;
  double matched_progress_s = 0.0;
  double remaining_progress_s = 0.0;
  double progress_delta_s = 0.0;

  double v_fraction = 0.0;
  double forward_v_fraction = 0.0;
  double omega_fraction = 0.0;
  double goal_slowdown = 0.0;

  double lateral_error = 0.0;
  double deviation_scale = 1.0;
  double lateral_ratio = 0.0;
  double path_heading_error = 0.0;
  double target_heading_error = 0.0;
  double tracking_heading_error = 0.0;
  double progress_gate = 0.0;
  double terminal_xy_gate = 0.0;
};
}  // namespace lt_dwa_v2_adapter
