#pragma once

#include <vector>

#include "lt_dwa_v2_adapter/core/planner_config.h"
#include "lt_dwa_v2_adapter/core/trajectory_types.h"

namespace lt_dwa_v2_adapter
{
class TrajectoryRollout
{
public:
  TrajectoryRollout() = default;
  explicit TrajectoryRollout(const RolloutConfig& config);

  void configure(const RolloutConfig& config);

  RobotState step(const RobotState& state, const Command& command) const;
  std::vector<RobotState> rollout(const RobotState& initial_state, const std::vector<Command>& commands) const;

  double dt() const;

private:
  RolloutConfig config_;
};
}  // namespace lt_dwa_v2_adapter
