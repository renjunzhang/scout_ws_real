#pragma once

#include <string>
#include <vector>

#include "lt_dwa_v2_adapter/core/planner_config.h"
#include "lt_dwa_v2_adapter/core/trajectory_types.h"
#include "lt_dwa_v2_adapter/geometry/path_reference.h"
#include "lt_dwa_v2_adapter/rollout/dynamic_window_sampler.h"
#include "lt_dwa_v2_adapter/rollout/trajectory_rollout.h"
#include "lt_dwa_v2_adapter/scoring/score_aggregator.h"
#include "lt_dwa_v2_adapter/search/frontier_pruner.h"
#include "lt_dwa_v2_adapter/world/occupancy_adapter.h"

namespace lt_dwa_v2_adapter
{
class LtDwaV2Planner
{
public:
  LtDwaV2Planner();
  explicit LtDwaV2Planner(const PlannerConfig& config);

  void configure(const PlannerConfig& config);

  PlanResult plan(const RobotState& current,
                  const std::vector<Pose2D>& path_points,
                  const OccupancyAdapter* occupancy,
                  double min_progress_s = 0.0,
                  double max_progress_s = -1.0) const;

private:
  PlannerConfig config_;
  DynamicWindowSampler sampler_;
  TrajectoryRollout rollout_;
  ScoreAggregator scorer_;
  FrontierPruner pruner_;

  Command pathTrackingSeed(const RobotState& state, const PathReference& path, double previous_progress_s) const;
  bool isGoalReached(const RobotState& state, const PathReference& path) const;
  bool collisionAt(const RobotState& state,
                   const OccupancyAdapter* occupancy,
                   CollisionDiagnostics* diagnostics = nullptr) const;
  void setStatus(PlanResult& result, PlannerStatusCode status) const;
};

std::string formatStatus(const PlanResult& result);
}  // namespace lt_dwa_v2_adapter
