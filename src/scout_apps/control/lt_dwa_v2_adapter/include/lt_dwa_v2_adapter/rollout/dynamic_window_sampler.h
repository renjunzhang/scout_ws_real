#pragma once

#include <vector>

#include "lt_dwa_v2_adapter/core/planner_config.h"
#include "lt_dwa_v2_adapter/core/trajectory_types.h"

namespace lt_dwa_v2_adapter
{
struct DynamicWindow
{
  double min_v = 0.0;
  double max_v = 0.0;
  double min_omega = 0.0;
  double max_omega = 0.0;
};

class DynamicWindowSampler
{
public:
  DynamicWindowSampler() = default;
  DynamicWindowSampler(const PlannerLimits& limits, const SamplingConfig& sampling, const RolloutConfig& rollout);

  void configure(const PlannerLimits& limits, const SamplingConfig& sampling, const RolloutConfig& rollout);

  DynamicWindow windowFor(const RobotState& state) const;
  std::vector<double> sampleLinearVelocities(double current_v) const;
  std::vector<double> sampleAngularVelocities(double current_omega) const;
  std::vector<Command> sampleCommands(const RobotState& state) const;

private:
  PlannerLimits limits_;
  SamplingConfig sampling_;
  RolloutConfig rollout_;
};
}  // namespace lt_dwa_v2_adapter
