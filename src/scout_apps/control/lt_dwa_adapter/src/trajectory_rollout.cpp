#include "lt_dwa_adapter/lt_dwa_planner.h"

#include <algorithm>
#include <cmath>

namespace lt_dwa_adapter
{
namespace
{
double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(hi, value));
}
}  // namespace

void LtDwaPlanner::configure(const PlannerConfig& config)
{
  config_ = config;
  config_.dt = std::max(0.02, config_.dt);
  config_.horizon_steps = std::max(1, config_.horizon_steps);
  config_.v_samples = std::max(1, config_.v_samples);
  config_.omega_samples = std::max(1, config_.omega_samples);
  config_.top_k_per_layer = std::max(1, config_.top_k_per_layer);
  config_.limits.v_max_mps = std::max(0.0, config_.limits.v_max_mps);
  config_.limits.omega_max_radps = std::max(0.0, config_.limits.omega_max_radps);
  config_.limits.a_max_mps2 = std::max(0.0, config_.limits.a_max_mps2);
  config_.limits.alpha_max_radps2 = std::max(0.0, config_.limits.alpha_max_radps2);
  config_.lookahead_distance_m = std::max(0.05, config_.lookahead_distance_m);
  config_.progress_rollback_tolerance_m = std::max(0.0, config_.progress_rollback_tolerance_m);
  config_.max_progress_advance_per_step_m = std::max(0.05, config_.max_progress_advance_per_step_m);
  config_.cross_track_heading_gain = std::max(0.0, config_.cross_track_heading_gain);
  config_.tracking_slowdown_lateral_m = std::max(0.05, config_.tracking_slowdown_lateral_m);
  config_.tracking_slowdown_heading_rad = std::max(0.05, config_.tracking_slowdown_heading_rad);
}

std::vector<double> LtDwaPlanner::sampleLinearVelocities(double current_v) const
{
  const double accel_step = config_.limits.a_max_mps2 * config_.dt;
  const double global_min = config_.limits.allow_reverse ? -config_.limits.v_max_mps : 0.0;
  const double lo = clamp(current_v - accel_step, global_min, config_.limits.v_max_mps);
  const double hi = clamp(current_v + accel_step, global_min, config_.limits.v_max_mps);

  std::vector<double> samples;
  if (config_.v_samples <= 1 || std::abs(hi - lo) < 1e-9)
  {
    samples.push_back(clamp(current_v, lo, hi));
    return samples;
  }
  samples.reserve(static_cast<size_t>(config_.v_samples));
  for (int i = 0; i < config_.v_samples; ++i)
  {
    const double r = static_cast<double>(i) / static_cast<double>(config_.v_samples - 1);
    samples.push_back(lo + r * (hi - lo));
  }
  return samples;
}

std::vector<double> LtDwaPlanner::sampleAngularVelocities(double current_w) const
{
  const double accel_step = config_.limits.alpha_max_radps2 * config_.dt;
  const double lo = clamp(current_w - accel_step, -config_.limits.omega_max_radps, config_.limits.omega_max_radps);
  const double hi = clamp(current_w + accel_step, -config_.limits.omega_max_radps, config_.limits.omega_max_radps);

  std::vector<double> samples;
  if (config_.omega_samples <= 1 || std::abs(hi - lo) < 1e-9)
  {
    samples.push_back(clamp(current_w, lo, hi));
    return samples;
  }
  samples.reserve(static_cast<size_t>(config_.omega_samples));
  for (int i = 0; i < config_.omega_samples; ++i)
  {
    const double r = static_cast<double>(i) / static_cast<double>(config_.omega_samples - 1);
    samples.push_back(lo + r * (hi - lo));
  }
  return samples;
}

std::vector<Command> LtDwaPlanner::sampleCommands(const RobotState& state) const
{
  const auto v_samples = sampleLinearVelocities(state.v);
  const auto omega_samples = sampleAngularVelocities(state.omega);
  std::vector<Command> commands;
  commands.reserve(v_samples.size() * omega_samples.size());
  for (const double v : v_samples)
  {
    for (const double omega : omega_samples)
    {
      Command cmd;
      cmd.v = v;
      cmd.omega = omega;
      commands.push_back(cmd);
    }
  }
  return commands;
}

RobotState LtDwaPlanner::rolloutStep(const RobotState& state, const Command& command) const
{
  RobotState next = state;
  next.v = command.v;
  next.omega = command.omega;
  next.x += command.v * std::cos(state.yaw) * config_.dt;
  next.y += command.v * std::sin(state.yaw) * config_.dt;
  next.yaw += command.omega * config_.dt;
  while (next.yaw > M_PI)
    next.yaw -= 2.0 * M_PI;
  while (next.yaw < -M_PI)
    next.yaw += 2.0 * M_PI;
  return next;
}
}  // namespace lt_dwa_adapter
