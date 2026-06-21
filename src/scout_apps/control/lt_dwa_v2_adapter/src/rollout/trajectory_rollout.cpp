#include "lt_dwa_v2_adapter/rollout/trajectory_rollout.h"

#include <algorithm>
#include <cmath>

namespace lt_dwa_v2_adapter
{
namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}
}  // namespace

TrajectoryRollout::TrajectoryRollout(const RolloutConfig& config)
{
  configure(config);
}

void TrajectoryRollout::configure(const RolloutConfig& config)
{
  config_ = config;
  config_.dt = std::max(0.02, config_.dt);
  config_.horizon_steps = std::max(1, config_.horizon_steps);
}

RobotState TrajectoryRollout::step(const RobotState& state, const Command& command) const
{
  RobotState next = state;
  next.v = command.v;
  next.omega = command.omega;
  next.x += command.v * std::cos(state.yaw) * config_.dt;
  next.y += command.v * std::sin(state.yaw) * config_.dt;
  next.yaw = normalizeAngle(state.yaw + command.omega * config_.dt);
  return next;
}

std::vector<RobotState> TrajectoryRollout::rollout(const RobotState& initial_state,
                                                   const std::vector<Command>& commands) const
{
  std::vector<RobotState> states;
  states.reserve(commands.size());
  RobotState state = initial_state;
  for (const Command& command : commands)
  {
    state = step(state, command);
    states.push_back(state);
  }
  return states;
}

double TrajectoryRollout::dt() const
{
  return config_.dt;
}
}  // namespace lt_dwa_v2_adapter
