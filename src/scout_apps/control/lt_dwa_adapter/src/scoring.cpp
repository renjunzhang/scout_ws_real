#include "lt_dwa_adapter/lt_dwa_planner.h"

#include <algorithm>
#include <cmath>

namespace lt_dwa_adapter
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

bool LtDwaPlanner::isGoalReached(const RobotState& state, const PathReference& path) const
{
  if (path.empty())
    return false;
  const Pose2D goal = path.sampleByProgress(path.totalLength());
  const double dist = std::hypot(goal.x - state.x, goal.y - state.y);
  const double yaw_err = std::abs(normalizeAngle(goal.yaw - state.yaw));
  return dist <= config_.goal_xy_tolerance_m && yaw_err <= config_.goal_yaw_tolerance_rad;
}

bool LtDwaPlanner::collisionAt(const RobotState& state,
                               const OccupancyAdapter* occupancy,
                               CollisionDiagnostics* diagnostics) const
{
  if (!occupancy)
  {
    if (diagnostics)
      *diagnostics = CollisionDiagnostics{};
    return false;
  }
  return occupancy->collisionAt(state, diagnostics);
}

double LtDwaPlanner::obstacleCost(const RobotState& state, const OccupancyAdapter* occupancy) const
{
  if (!occupancy)
    return 0.0;
  return occupancy->obstacleCost(state);
}

double LtDwaPlanner::scorePoint(const RobotState& state,
                                const Command& command,
                                const Command& previous_command,
                                double previous_progress_s,
                                const PathReference& path,
                                const OccupancyAdapter* occupancy,
                                ScoreBreakdown& score) const
{
  const double min_progress_s = std::max(0.0, previous_progress_s - config_.progress_rollback_tolerance_m);
  const double max_progress_s = previous_progress_s + config_.max_progress_advance_per_step_m;
  const PathProjection match = path.project(state, min_progress_s, max_progress_s);
  const Pose2D goal = path.sampleByProgress(path.totalLength());
  const double terminal_dist = std::hypot(goal.x - state.x, goal.y - state.y);
  const double path_scale = std::max(1.0, path.totalLength());
  const double matched_progress_s = match.valid ? match.progress_s : previous_progress_s;
  const double remaining_progress_s = std::max(0.0, path.totalLength() - matched_progress_s);
  const double progress_delta_s = match.valid ? match.progress_s - previous_progress_s : 0.0;
  const double v_fraction = config_.limits.v_max_mps > 1e-9 ? command.v / config_.limits.v_max_mps : 0.0;
  const double forward_v_fraction = std::max(0.0, v_fraction);
  const double goal_slowdown = std::max(0.0, std::min(1.0, terminal_dist / 0.80));
  const double lateral_error = match.valid ? match.distance : config_.max_tracking_deviation_m;
  const double deviation_scale = std::max(0.20, config_.max_tracking_deviation_m);
  const double lateral_ratio = lateral_error / deviation_scale;
  const double path_heading_error = match.valid ? match.heading_error : M_PI;
  const Pose2D target = path.sampleByProgress((match.valid ? match.progress_s : previous_progress_s) +
                                              config_.lookahead_distance_m);
  const double target_heading = std::atan2(target.y - state.y, target.x - state.x);
  const double target_heading_error = std::abs(normalizeAngle(target_heading - state.yaw));
  const double tracking_heading_error = std::max(path_heading_error, target_heading_error);
  const double omega_fraction = config_.limits.omega_max_radps > 1e-9 ?
                                  std::abs(command.omega) / config_.limits.omega_max_radps :
                                  0.0;

  score.obstacle = config_.weights.obstacle * obstacleCost(state, occupancy);
  score.path_lateral = config_.weights.path_lateral * lateral_error * (1.0 + 3.0 * lateral_ratio * lateral_ratio);
  if (lateral_error > config_.tracking_slowdown_lateral_m)
  {
    const double lateral_excess = lateral_error - config_.tracking_slowdown_lateral_m;
    score.path_lateral += config_.weights.path_lateral * lateral_excess * lateral_excess /
                          std::max(0.05, config_.tracking_slowdown_lateral_m);
  }
  score.heading = config_.weights.heading * path_heading_error * (1.0 + path_heading_error / M_PI) +
                  0.75 * config_.weights.heading * target_heading_error;
  const double progress_gate = std::max(0.0, std::min(1.0, 1.0 - lateral_ratio)) *
                               std::max(0.0, std::cos(std::min(M_PI / 2.0, target_heading_error)));
  score.progress = -config_.weights.progress * std::max(0.0, progress_delta_s) * progress_gate / path_scale;
  if (progress_delta_s < -0.05)
    score.progress += config_.weights.progress * std::abs(progress_delta_s) / path_scale;
  if (lateral_error > 0.80 * deviation_scale)
    score.progress += config_.weights.progress * (lateral_error - 0.80 * deviation_scale) / deviation_scale;
  const double terminal_xy_gate = std::max(0.0, std::min(1.0, 1.0 - remaining_progress_s / 1.0));
  score.terminal = config_.weights.terminal * remaining_progress_s / path_scale +
                   terminal_xy_gate * config_.weights.terminal * terminal_dist;
  score.smooth = config_.weights.smooth_v * std::abs(command.v - previous_command.v) +
                 config_.weights.smooth_omega * std::abs(command.omega - previous_command.omega) +
                 0.25 * config_.weights.smooth_omega * std::abs(command.omega);
  score.speed = -config_.weights.speed * forward_v_fraction * goal_slowdown * progress_gate;
  const double lateral_slowdown_excess = std::max(0.0, lateral_error - config_.tracking_slowdown_lateral_m) /
                                         std::max(0.05, config_.tracking_slowdown_lateral_m);
  const double heading_slowdown_excess = std::max(0.0, tracking_heading_error - config_.tracking_slowdown_heading_rad);
  score.speed += config_.weights.speed * forward_v_fraction *
                 (2.0 * lateral_slowdown_excess + 1.5 * heading_slowdown_excess);
  if (lateral_error < config_.tracking_slowdown_lateral_m &&
      tracking_heading_error < config_.tracking_slowdown_heading_rad)
  {
    const double heading_alignment = 1.0 - tracking_heading_error / config_.tracking_slowdown_heading_rad;
    score.smooth += 0.50 * config_.weights.heading * omega_fraction * std::max(0.0, heading_alignment);
    if (remaining_progress_s > config_.goal_xy_tolerance_m && command.v < 0.03)
      score.speed += 2.0 * config_.weights.speed * (1.0 + remaining_progress_s / path_scale);
  }
  if (terminal_dist < 0.80)
    score.speed += 4.0 * config_.weights.speed * (1.0 - goal_slowdown) * forward_v_fraction;
  return weightedTotal(score);
}

void LtDwaPlanner::accumulateScore(ScoreBreakdown& total, const ScoreBreakdown& inc) const
{
  total.obstacle += inc.obstacle;
  total.path_lateral += inc.path_lateral;
  total.heading += inc.heading;
  total.progress += inc.progress;
  total.terminal += inc.terminal;
  total.smooth += inc.smooth;
  total.speed += inc.speed;
}

double LtDwaPlanner::weightedTotal(const ScoreBreakdown& score) const
{
  return score.obstacle + score.path_lateral + score.heading + score.progress + score.terminal + score.smooth + score.speed;
}
}  // namespace lt_dwa_adapter
