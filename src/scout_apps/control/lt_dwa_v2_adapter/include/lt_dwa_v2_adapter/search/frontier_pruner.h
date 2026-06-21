#pragma once

#include <vector>

#include "lt_dwa_v2_adapter/core/planner_config.h"
#include "lt_dwa_v2_adapter/core/trajectory_types.h"

namespace lt_dwa_v2_adapter
{
class FrontierPruner
{
public:
  FrontierPruner() = default;
  explicit FrontierPruner(const SearchConfig& config);

  void configure(const SearchConfig& config);
  void prune(std::vector<TrajectoryCandidate>& frontier) const;
  const TrajectoryCandidate* best(const std::vector<TrajectoryCandidate>& frontier) const;

private:
  bool better(const TrajectoryCandidate& a, const TrajectoryCandidate& b) const;

  SearchConfig config_;
};
}  // namespace lt_dwa_v2_adapter
