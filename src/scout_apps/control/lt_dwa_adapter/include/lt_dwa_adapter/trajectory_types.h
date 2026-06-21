#pragma once

#include <limits>
#include <string>
#include <vector>

namespace lt_dwa_adapter
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

struct PlannerLimits
{
  double v_max_mps = 0.8;
  double omega_max_radps = 1.2;
  double a_max_mps2 = 0.6;
  double alpha_max_radps2 = 1.2;
  bool allow_reverse = false;
};

struct PlannerWeights
{
  double obstacle = 2.0;
  double path_lateral = 4.0;
  double heading = 2.0;
  double progress = 3.0;
  double terminal = 4.0;
  double smooth_v = 0.0;
  double smooth_omega = 0.08;
  double speed = 1.0;
};

struct PlannerConfig
{
  PlannerLimits limits;
  PlannerWeights weights;
  double dt = 0.15;
  int horizon_steps = 12;
  int v_samples = 7;
  int omega_samples = 9;
  int top_k_per_layer = 80;
  double robot_radius_m = 0.35;
  double clearance_radius_m = 0.80;
  int lethal_occupancy = 65;
  bool treat_unknown_as_occupied = false;
  double goal_xy_tolerance_m = 0.20;
  double goal_yaw_tolerance_rad = 0.30;
  double max_tracking_deviation_m = 1.50;
  double lookahead_distance_m = 0.55;
  double progress_rollback_tolerance_m = 0.35;
  double max_progress_advance_per_step_m = 0.35;
  double cross_track_heading_gain = 1.40;
  double tracking_slowdown_lateral_m = 0.35;
  double tracking_slowdown_heading_rad = 0.65;
};

struct ScoreBreakdown
{
  double obstacle = 0.0;
  double path_lateral = 0.0;
  double heading = 0.0;
  double progress = 0.0;
  double terminal = 0.0;
  double smooth = 0.0;
  double speed = 0.0;
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
  std::string status = "NO_VALID_CMD";
  Command command;
  TrajectoryCandidate best;
  PlanDiagnostics diagnostics;
  int expanded_nodes = 0;
  int valid_candidates = 0;
};
}  // namespace lt_dwa_adapter
