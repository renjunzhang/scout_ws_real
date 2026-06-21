#pragma once

#include <limits>
#include <string>
#include <vector>

#include "lt_dwa_v2_adapter/core/planner_status.h"
#include "lt_dwa_v2_adapter/scoring/score_breakdown.h"

namespace lt_dwa_v2_adapter
{
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct RobotState
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  double omega = 0.0;
};

struct Command
{
  double v = 0.0;
  double omega = 0.0;
};

struct CollisionDiagnostics
{
  int checked_samples = 0;
  int unknown_samples = 0;
  int out_of_map_samples = 0;
  int lethal_samples = 0;
  int center_occupancy = -1;
  int max_occupancy = -1;
  bool has_first_lethal_sample = false;
  double first_lethal_x = 0.0;
  double first_lethal_y = 0.0;
};

struct PlanDiagnostics
{
  bool has_initial_match = false;
  double initial_match_index = 0.0;
  double initial_match_distance = 0.0;
  double initial_signed_lateral_error = 0.0;
  double initial_match_heading_error = 0.0;
  double initial_progress_s = 0.0;
  int lookahead_target_index = -1;
  double lookahead_target_x = 0.0;
  double lookahead_target_y = 0.0;
  double lookahead_target_progress_s = 0.0;
  bool tracking_diverged = false;
  double max_tracking_deviation_m = 0.0;
  bool plan_map_transform_ok = true;
  bool initial_collision = false;
  CollisionDiagnostics initial_collision_details;
};

struct TrajectoryPoint
{
  RobotState state;
  Command command;
  ScoreBreakdown score;
  double incremental_cost = 0.0;
};

struct TrajectoryCandidate
{
  std::vector<TrajectoryPoint> points;
  Command first_command;
  ScoreBreakdown score;
  double total_cost = std::numeric_limits<double>::infinity();
  double progress_index = 0.0;
  double progress_s = 0.0;
  bool valid = false;
};

struct PlanResult
{
  bool valid = false;
  PlannerStatusCode status_code = PlannerStatusCode::NoValidCommand;
  std::string status = statusString(status_code);
  Command command;
  TrajectoryCandidate best;
  PlanDiagnostics diagnostics;
  int expanded_nodes = 0;
  int valid_candidates = 0;
};
}  // namespace lt_dwa_v2_adapter
