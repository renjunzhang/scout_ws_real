#include "lt_dwa_v2_adapter/scoring/tracking_cost.h"

#include <algorithm>
#include <cmath>

namespace lt_dwa_v2_adapter
{
TrackingCostTerms evaluateTrackingCosts(const ScoreContext& context)
{
  TrackingCostTerms terms;
  if (!context.config)
    return terms;

  const PlannerConfig& config = *context.config;
  terms.path_lateral = config.weights.path_lateral * context.lateral_error *
                       (1.0 + 3.0 * context.lateral_ratio * context.lateral_ratio);
  if (context.lateral_error > config.tracking.tracking_slowdown_lateral_m)
  {
    const double lateral_excess = context.lateral_error - config.tracking.tracking_slowdown_lateral_m;
    terms.path_lateral += config.weights.path_lateral * lateral_excess * lateral_excess /
                          std::max(0.05, config.tracking.tracking_slowdown_lateral_m);
  }

  terms.heading = config.weights.heading * context.path_heading_error *
                      (1.0 + context.path_heading_error / M_PI) +
                  0.75 * config.weights.heading * context.target_heading_error;

  terms.progress = -config.weights.progress * std::max(0.0, context.progress_delta_s) *
                   context.progress_gate / context.path_scale;
  if (context.progress_delta_s < -0.05)
    terms.progress += config.weights.progress * std::abs(context.progress_delta_s) / context.path_scale;
  if (context.lateral_error > 0.80 * context.deviation_scale)
  {
    terms.progress += config.weights.progress *
                      (context.lateral_error - 0.80 * context.deviation_scale) / context.deviation_scale;
  }
  return terms;
}
}  // namespace lt_dwa_v2_adapter
