#pragma once

#include "lt_dwa_v2_adapter/scoring/score_context.h"

namespace lt_dwa_v2_adapter
{
struct TrackingCostTerms
{
  double path_lateral = 0.0;
  double heading = 0.0;
  double progress = 0.0;
};

TrackingCostTerms evaluateTrackingCosts(const ScoreContext& context);
}  // namespace lt_dwa_v2_adapter
