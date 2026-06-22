#include "lt_dwa_v2_adapter/scoring/speed_cost.h"

#include <algorithm>

namespace lt_dwa_v2_adapter
{
double evaluateSpeedCost(const ScoreContext& context)
{
  if (!context.config)
    return 0.0;

  const PlannerConfig& config = *context.config;
  double cost = -config.weights.speed * context.forward_v_fraction * context.goal_slowdown * context.progress_gate;

  const double lateral_slowdown_excess =
      std::max(0.0, context.lateral_error - config.tracking.tracking_slowdown_lateral_m) /
      std::max(0.05, config.tracking.tracking_slowdown_lateral_m);
  const double heading_slowdown_excess =
      std::max(0.0, context.tracking_heading_error - config.tracking.tracking_slowdown_heading_rad);
  cost += config.weights.speed * context.forward_v_fraction *
          (3.0 * lateral_slowdown_excess + 3.0 * heading_slowdown_excess);
  if (context.tracking_heading_error > config.tracking.tracking_slowdown_heading_rad)
  {
    const double heading_ratio =
        (context.tracking_heading_error - config.tracking.tracking_slowdown_heading_rad) /
        std::max(0.05, config.tracking.tracking_slowdown_heading_rad);
    cost += 4.0 * config.weights.speed * context.forward_v_fraction * context.forward_v_fraction * heading_ratio;
  }
  if (context.tracking_heading_error > 1.20)
    cost += 6.0 * config.weights.speed * context.forward_v_fraction * (context.tracking_heading_error - 1.20);

  if (context.remaining_progress_s > config.goal.xy_tolerance_m)
  {
    const double low_forward_factor = std::max(0.0, 1.0 - context.forward_v_fraction / 0.25);
    cost += config.weights.speed * (1.0 + context.lateral_ratio) * low_forward_factor;
    cost += 3.0 * config.weights.speed * context.omega_fraction * context.omega_fraction * low_forward_factor;
  }

  if (context.lateral_error < config.tracking.tracking_slowdown_lateral_m &&
      context.tracking_heading_error < config.tracking.tracking_slowdown_heading_rad)
  {
    if (context.remaining_progress_s > config.goal.xy_tolerance_m && context.command.v < 0.03)
      cost += 2.0 * config.weights.speed * (1.0 + context.remaining_progress_s / context.path_scale);
  }
  if (context.terminal_dist < 0.80)
    cost += 4.0 * config.weights.speed * (1.0 - context.goal_slowdown) * context.forward_v_fraction;
  if (context.remaining_progress_s < 0.70 && context.terminal_dist < 0.65)
  {
    cost += 4.0 * config.weights.speed * context.omega_fraction * context.omega_fraction;
    if (context.terminal_dist > config.goal.xy_tolerance_m && context.target_heading_error < 0.70 && context.command.v < 0.06)
      cost += 4.0 * config.weights.speed * (1.0 - context.command.v / 0.06);
  }
  if (context.terminal_dist < 0.35)
  {
    cost += 10.0 * config.weights.speed * context.omega_fraction * context.omega_fraction;
    cost += 6.0 * config.weights.speed * context.forward_v_fraction * context.forward_v_fraction;
  }
  return cost;
}
}  // namespace lt_dwa_v2_adapter
