#include "lt_dwa_v2_adapter/scoring/obstacle_cost.h"

namespace lt_dwa_v2_adapter
{
double evaluateObstacleCost(const ScoreContext& context)
{
  if (!context.config || !context.occupancy)
    return 0.0;
  return context.config->weights.obstacle * context.occupancy->obstacleCost(context.state);
}
}  // namespace lt_dwa_v2_adapter
