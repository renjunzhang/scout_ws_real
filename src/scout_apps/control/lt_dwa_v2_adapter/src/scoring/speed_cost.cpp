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
          (2.0 * lateral_slowdown_excess + 1.5 * heading_slowdown_excess);

  if (context.lateral_error < config.tracking.tracking_slowdown_lateral_m &&
      context.tracking_heading_error < config.tracking.tracking_slowdown_heading_rad)
  {
    if (context.remaining_progress_s > config.goal.xy_tolerance_m && context.command.v < 0.03)
      cost += 2.0 * config.weights.speed * (1.0 + context.remaining_progress_s / context.path_scale);
  }
  if (context.terminal_dist < 0.80)
    cost += 4.0 * config.weights.speed * (1.0 - context.goal_slowdown) * context.forward_v_fraction;
  return cost;
}
}  // namespace lt_dwa_v2_adapter
