#include "lt_dwa_v2_adapter/ros/parameter_loader.h"

#include <algorithm>
#include <cmath>

namespace lt_dwa_v2_adapter
{
namespace
{
template <typename T>
void loadParam(const ros::NodeHandle& nh, const std::string& name, T& value)
{
  nh.param(name, value, value);
}

void sanitize(PlannerConfig& config)
{
  if (!std::isfinite(config.timing.planning_frequency) || config.timing.planning_frequency <= 0.0)
    config.timing.planning_frequency = 10.0;
  if (!std::isfinite(config.timing.command_publish_frequency) || config.timing.command_publish_frequency <= 0.0)
    config.timing.command_publish_frequency = 25.0;
  if (!std::isfinite(config.timing.command_stale_timeout_sec) || config.timing.command_stale_timeout_sec <= 0.0)
    config.timing.command_stale_timeout_sec = 2.0;
  if (!std::isfinite(config.frames.tf_timeout_sec) || config.frames.tf_timeout_sec < 0.0)
    config.frames.tf_timeout_sec = 0.2;

  config.limits.v_max_mps = std::max(0.0, config.limits.v_max_mps);
  config.limits.omega_max_radps = std::max(0.0, config.limits.omega_max_radps);
  config.limits.a_max_mps2 = std::max(0.0, config.limits.a_max_mps2);
  config.limits.alpha_max_radps2 = std::max(0.0, config.limits.alpha_max_radps2);

  config.goal.xy_tolerance_m = std::max(0.0, config.goal.xy_tolerance_m);
  config.goal.yaw_tolerance_rad = std::max(0.0, config.goal.yaw_tolerance_rad);

  config.tracking.max_tracking_deviation_m = std::max(0.0, config.tracking.max_tracking_deviation_m);
  config.tracking.lookahead_distance_m = std::max(0.05, config.tracking.lookahead_distance_m);
  config.tracking.progress_rollback_tolerance_m = std::max(0.0, config.tracking.progress_rollback_tolerance_m);
  config.tracking.max_progress_advance_per_step_m = std::max(0.05, config.tracking.max_progress_advance_per_step_m);
  config.tracking.cross_track_heading_gain = std::max(0.0, config.tracking.cross_track_heading_gain);
  config.tracking.tracking_slowdown_lateral_m = std::max(0.05, config.tracking.tracking_slowdown_lateral_m);
  config.tracking.tracking_slowdown_heading_rad = std::max(0.05, config.tracking.tracking_slowdown_heading_rad);

  config.rollout.dt = std::max(0.02, config.rollout.dt);
  config.rollout.horizon_steps = std::max(1, config.rollout.horizon_steps);
  config.sampling.v_samples = std::max(1, config.sampling.v_samples);
  config.sampling.omega_samples = std::max(1, config.sampling.omega_samples);
  config.search.top_k_per_layer = std::max(1, config.search.top_k_per_layer);

  config.occupancy.robot_radius_m = std::max(0.01, config.occupancy.robot_radius_m);
  config.occupancy.clearance_radius_m = std::max(config.occupancy.robot_radius_m, config.occupancy.clearance_radius_m);
  config.occupancy.lethal_occupancy = std::max(0, std::min(100, config.occupancy.lethal_occupancy));
}
}  // namespace

PlannerConfig loadPlannerConfig(const ros::NodeHandle& private_nh)
{
  PlannerConfig config;

  loadParam(private_nh, "runtime/publish_cmd_vel", config.runtime.publish_cmd_vel);
  loadParam(private_nh, "runtime/publish_shadow_cmd", config.runtime.publish_shadow_cmd);
  loadParam(private_nh, "runtime/publish_diagnostics", config.runtime.publish_diagnostics);
  loadParam(private_nh, "runtime/require_map", config.runtime.require_map);

  loadParam(private_nh, "topics/odom_topic", config.topics.odom_topic);
  loadParam(private_nh, "topics/map_topic", config.topics.map_topic);
  loadParam(private_nh, "topics/global_path_topic", config.topics.global_path_topic);
  loadParam(private_nh, "topics/goal_topic", config.topics.goal_topic);
  loadParam(private_nh, "topics/cmd_vel_topic", config.topics.cmd_vel_topic);
  loadParam(private_nh, "topics/shadow_cmd_topic", config.topics.shadow_cmd_topic);
  loadParam(private_nh, "topics/status_topic", config.topics.status_topic);
  loadParam(private_nh, "topics/diagnostics_topic", config.topics.diagnostics_topic);
  loadParam(private_nh, "topics/global_plan_topic", config.topics.global_plan_topic);
  loadParam(private_nh, "topics/local_plan_topic", config.topics.local_plan_topic);

  loadParam(private_nh, "frames/base_frame", config.frames.base_frame);
  loadParam(private_nh, "frames/plan_target_frame", config.frames.plan_target_frame);
  loadParam(private_nh, "frames/tf_timeout_sec", config.frames.tf_timeout_sec);

  loadParam(private_nh, "timing/planning_frequency", config.timing.planning_frequency);
  loadParam(private_nh, "timing/command_publish_frequency", config.timing.command_publish_frequency);
  loadParam(private_nh, "timing/command_stale_timeout_sec", config.timing.command_stale_timeout_sec);

  loadParam(private_nh, "limits/v_max_mps", config.limits.v_max_mps);
  loadParam(private_nh, "limits/omega_max_radps", config.limits.omega_max_radps);
  loadParam(private_nh, "limits/a_max_mps2", config.limits.a_max_mps2);
  loadParam(private_nh, "limits/alpha_max_radps2", config.limits.alpha_max_radps2);
  loadParam(private_nh, "limits/allow_reverse", config.limits.allow_reverse);

  loadParam(private_nh, "goal/xy_tolerance_m", config.goal.xy_tolerance_m);
  loadParam(private_nh, "goal/yaw_tolerance_rad", config.goal.yaw_tolerance_rad);

  loadParam(private_nh, "tracking/max_tracking_deviation_m", config.tracking.max_tracking_deviation_m);
  loadParam(private_nh, "tracking/lookahead_distance_m", config.tracking.lookahead_distance_m);
  loadParam(private_nh, "tracking/progress_rollback_tolerance_m", config.tracking.progress_rollback_tolerance_m);
  loadParam(private_nh, "tracking/max_progress_advance_per_step_m", config.tracking.max_progress_advance_per_step_m);
  loadParam(private_nh, "tracking/cross_track_heading_gain", config.tracking.cross_track_heading_gain);
  loadParam(private_nh, "tracking/tracking_slowdown_lateral_m", config.tracking.tracking_slowdown_lateral_m);
  loadParam(private_nh, "tracking/tracking_slowdown_heading_rad", config.tracking.tracking_slowdown_heading_rad);

  loadParam(private_nh, "rollout/dt", config.rollout.dt);
  loadParam(private_nh, "rollout/horizon_steps", config.rollout.horizon_steps);

  loadParam(private_nh, "sampling/v_samples", config.sampling.v_samples);
  loadParam(private_nh, "sampling/omega_samples", config.sampling.omega_samples);

  loadParam(private_nh, "search/top_k_per_layer", config.search.top_k_per_layer);
  loadParam(private_nh, "search/deterministic_tie_break", config.search.deterministic_tie_break);

  loadParam(private_nh, "occupancy/robot_radius_m", config.occupancy.robot_radius_m);
  loadParam(private_nh, "occupancy/clearance_radius_m", config.occupancy.clearance_radius_m);
  loadParam(private_nh, "occupancy/lethal_occupancy", config.occupancy.lethal_occupancy);
  loadParam(private_nh, "occupancy/treat_unknown_as_occupied", config.occupancy.treat_unknown_as_occupied);

  loadParam(private_nh, "weights/obstacle", config.weights.obstacle);
  loadParam(private_nh, "weights/path_lateral", config.weights.path_lateral);
  loadParam(private_nh, "weights/heading", config.weights.heading);
  loadParam(private_nh, "weights/progress", config.weights.progress);
  loadParam(private_nh, "weights/terminal", config.weights.terminal);
  loadParam(private_nh, "weights/smooth_v", config.weights.smooth_v);
  loadParam(private_nh, "weights/smooth_omega", config.weights.smooth_omega);
  loadParam(private_nh, "weights/speed", config.weights.speed);

  sanitize(config);
  return config;
}
}  // namespace lt_dwa_v2_adapter
