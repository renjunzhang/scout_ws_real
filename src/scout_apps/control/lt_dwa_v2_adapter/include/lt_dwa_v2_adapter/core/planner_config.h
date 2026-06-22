#pragma once

#include <string>

namespace lt_dwa_v2_adapter
{
struct RuntimeConfig
{
  bool publish_cmd_vel = false;
  bool publish_shadow_cmd = true;
  bool publish_diagnostics = false;
  bool require_map = true;
};

struct TopicConfig
{
  std::string odom_topic = "/odom";
  std::string map_topic = "/map";
  std::string global_path_topic = "/scout/global_path_fixed";
  std::string goal_topic = "/scout/goal";
  std::string cmd_vel_topic = "/lt_dwa_v2/shadow_cmd_vel";
  std::string shadow_cmd_topic = "/baseline/lt_dwa_v2/shadow_cmd_vel";
  std::string status_topic = "/baseline/lt_dwa_v2/status";
  std::string diagnostics_topic = "/baseline/lt_dwa_v2/diagnostics";
  std::string global_plan_topic = "/baseline/lt_dwa_v2/global_plan";
  std::string local_plan_topic = "/baseline/lt_dwa_v2/local_plan";
};

struct FrameConfig
{
  std::string base_frame = "base_link";
  std::string plan_target_frame = "odom";
  double tf_timeout_sec = 0.2;
};

struct TimingConfig
{
  double planning_frequency = 10.0;
  double command_publish_frequency = 25.0;
  double command_stale_timeout_sec = 2.0;
};

struct PlannerLimits
{
  double v_max_mps = 0.8;
  double omega_max_radps = 1.2;
  double a_max_mps2 = 0.6;
  double alpha_max_radps2 = 1.2;
  bool allow_reverse = false;
};

struct GoalConfig
{
  double xy_tolerance_m = 0.20;
  double yaw_tolerance_rad = 0.30;
};

struct TrackingConfig
{
  double max_tracking_deviation_m = 1.50;
  double lookahead_distance_m = 0.35;
  double progress_rollback_tolerance_m = 0.35;
  double max_progress_advance_per_step_m = 0.20;
  double cross_track_heading_gain = 1.40;
  double tracking_slowdown_lateral_m = 0.35;
  double tracking_slowdown_heading_rad = 0.65;
};

struct RolloutConfig
{
  double dt = 0.15;
  int horizon_steps = 12;
};

struct SamplingConfig
{
  int v_samples = 7;
  int omega_samples = 9;
};

struct SearchConfig
{
  int top_k_per_layer = 80;
  std::string deterministic_tie_break = "progress_then_cost";
};

struct OccupancyConfig
{
  double robot_radius_m = 0.35;
  double clearance_radius_m = 0.80;
  int lethal_occupancy = 65;
  bool treat_unknown_as_occupied = false;
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
  RuntimeConfig runtime;
  TopicConfig topics;
  FrameConfig frames;
  TimingConfig timing;
  PlannerLimits limits;
  GoalConfig goal;
  TrackingConfig tracking;
  RolloutConfig rollout;
  SamplingConfig sampling;
  SearchConfig search;
  OccupancyConfig occupancy;
  PlannerWeights weights;
};
}  // namespace lt_dwa_v2_adapter
