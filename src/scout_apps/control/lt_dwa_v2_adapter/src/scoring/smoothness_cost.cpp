#include "lt_dwa_v2_adapter/scoring/smoothness_cost.h"

#include <algorithm>
#include <cmath>

namespace lt_dwa_v2_adapter
{
double evaluateSmoothnessCost(const ScoreContext& context)
{
  if (!context.config)
    return 0.0;

  const PlannerConfig& config = *context.config;
  double cost = config.weights.smooth_v * std::abs(context.command.v - context.previous_command.v) +
                config.weights.smooth_omega * std::abs(context.command.omega - context.previous_command.omega) +
                0.25 * config.weights.smooth_omega * std::abs(context.command.omega);

  if (context.lateral_error < config.tracking.tracking_slowdown_lateral_m &&
      context.tracking_heading_error < config.tracking.tracking_slowdown_heading_rad)
  {
    const double heading_alignment =
        1.0 - context.tracking_heading_error / config.tracking.tracking_slowdown_heading_rad;
    cost += 0.50 * config.weights.heading * context.omega_fraction * std::max(0.0, heading_alignment);
  }
  return cost;
}
}  // namespace lt_dwa_v2_adapter
