#pragma once

#include "lt_dwa_v2_adapter/core/planner_config.h"
#include "lt_dwa_v2_adapter/geometry/path_reference.h"
#include "lt_dwa_v2_adapter/scoring/score_breakdown.h"
#include "lt_dwa_v2_adapter/scoring/score_context.h"
#include "lt_dwa_v2_adapter/world/occupancy_adapter.h"

namespace lt_dwa_v2_adapter
{
class ScoreAggregator
{
public:
  ScoreAggregator() = default;
  explicit ScoreAggregator(const PlannerConfig& config);

  void configure(const PlannerConfig& config);

  ScoreContext makeContext(const RobotState& state,
                           const Command& command,
                           const Command& previous_command,
                           double previous_progress_s,
                           const PathReference& path,
                           const OccupancyAdapter* occupancy) const;

  double scorePoint(const RobotState& state,
                    const Command& command,
                    const Command& previous_command,
                    double previous_progress_s,
                    const PathReference& path,
                    const OccupancyAdapter* occupancy,
                    ScoreBreakdown& score) const;

  void accumulateScore(ScoreBreakdown& total, const ScoreBreakdown& inc) const;
  double weightedTotal(const ScoreBreakdown& score) const;

private:
  PlannerConfig config_;
};
}  // namespace lt_dwa_v2_adapter
