#include "lt_dwa_v2_adapter/scoring/terminal_cost.h"

namespace lt_dwa_v2_adapter
{
double evaluateTerminalCost(const ScoreContext& context)
{
  if (!context.config)
    return 0.0;
  return context.config->weights.terminal * context.remaining_progress_s / context.path_scale +
         context.terminal_xy_gate * context.config->weights.terminal * context.terminal_dist;
}
}  // namespace lt_dwa_v2_adapter
