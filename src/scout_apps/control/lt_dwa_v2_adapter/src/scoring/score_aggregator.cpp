#include "lt_dwa_v2_adapter/scoring/score_aggregator.h"

#include <algorithm>
#include <cmath>

#include "lt_dwa_v2_adapter/scoring/obstacle_cost.h"
#include "lt_dwa_v2_adapter/scoring/smoothness_cost.h"
#include "lt_dwa_v2_adapter/scoring/speed_cost.h"
#include "lt_dwa_v2_adapter/scoring/terminal_cost.h"
#include "lt_dwa_v2_adapter/scoring/tracking_cost.h"

namespace lt_dwa_v2_adapter
{
namespace
{
double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}
}  // namespace

ScoreAggregator::ScoreAggregator(const PlannerConfig& config)
{
  configure(config);
}

void ScoreAggregator::configure(const PlannerConfig& config)
{
  config_ = config;
  config_.tracking.progress_rollback_tolerance_m = std::max(0.0, config_.tracking.progress_rollback_tolerance_m);
  config_.tracking.max_progress_advance_per_step_m = std::max(0.05, config_.tracking.max_progress_advance_per_step_m);
  config_.tracking.max_tracking_deviation_m = std::max(0.20, config_.tracking.max_tracking_deviation_m);
  config_.tracking.lookahead_distance_m = std::max(0.05, config_.tracking.lookahead_distance_m);
  config_.tracking.tracking_slowdown_lateral_m = std::max(0.05, config_.tracking.tracking_slowdown_lateral_m);
  config_.tracking.tracking_slowdown_heading_rad = std::max(0.05, config_.tracking.tracking_slowdown_heading_rad);
}

ScoreContext ScoreAggregator::makeContext(const RobotState& state,
                                          const Command& command,
                                          const Command& previous_command,
                                          double previous_progress_s,
                                          const PathReference& path,
                                          const OccupancyAdapter* occupancy) const
{
  ScoreContext context;
  context.config = &config_;
  context.path = &path;
  context.occupancy = occupancy;
  context.state = state;
  context.command = command;
  context.previous_command = previous_command;
  context.previous_progress_s = previous_progress_s;

  const double min_progress_s = std::max(0.0, previous_progress_s);
  const double max_progress_s = previous_progress_s + config_.tracking.max_progress_advance_per_step_m;
  context.match = path.project(state, min_progress_s, max_progress_s);
  context.goal = path.sampleByProgress(path.totalLength());
  context.terminal_dist = std::hypot(context.goal.x - state.x, context.goal.y - state.y);
  context.path_scale = std::max(1.0, path.totalLength());
  context.matched_progress_s = context.match.valid ? context.match.progress_s : previous_progress_s;
  context.remaining_progress_s = std::max(0.0, path.totalLength() - context.matched_progress_s);
  context.progress_delta_s = context.match.valid ? context.match.progress_s - previous_progress_s : 0.0;

  context.v_fraction = config_.limits.v_max_mps > 1e-9 ? command.v / config_.limits.v_max_mps : 0.0;
  context.forward_v_fraction = std::max(0.0, context.v_fraction);
  context.omega_fraction = config_.limits.omega_max_radps > 1e-9 ?
                               std::abs(command.omega) / config_.limits.omega_max_radps :
                               0.0;
  context.goal_slowdown = clamp01(context.terminal_dist / 0.80);

  context.lateral_error = context.match.valid ? context.match.distance : config_.tracking.max_tracking_deviation_m;
  context.deviation_scale = std::max(0.20, config_.tracking.max_tracking_deviation_m);
  context.lateral_ratio = context.lateral_error / context.deviation_scale;
  context.path_heading_error = context.match.valid ? context.match.heading_error : M_PI;
  context.target = path.sampleByProgress((context.match.valid ? context.match.progress_s : previous_progress_s) +
                                         config_.tracking.lookahead_distance_m);
  const double target_heading = std::atan2(context.target.y - state.y, context.target.x - state.x);
  context.target_heading_error = std::abs(normalizeAngle(target_heading - state.yaw));
  context.terminal_xy_gate = clamp01(1.0 - context.remaining_progress_s / 1.0);
  context.tracking_heading_error = context.terminal_xy_gate > 0.5 ? context.target_heading_error :
                                                                  std::max(context.path_heading_error,
                                                                           context.target_heading_error);
  const double lateral_progress_gate = clamp01(1.0 - 0.70 * context.lateral_ratio);
  const double heading_progress_gate = clamp01(std::cos(std::min(M_PI / 2.0, context.target_heading_error)));
  context.progress_gate = std::max(0.25, lateral_progress_gate) * std::max(0.20, heading_progress_gate);
  return context;
}

double ScoreAggregator::scorePoint(const RobotState& state,
                                   const Command& command,
                                   const Command& previous_command,
                                   double previous_progress_s,
                                   const PathReference& path,
                                   const OccupancyAdapter* occupancy,
                                   ScoreBreakdown& score) const
{
  const ScoreContext context = makeContext(state, command, previous_command, previous_progress_s, path, occupancy);
  score = ScoreBreakdown{};
  score.obstacle = evaluateObstacleCost(context);
  const TrackingCostTerms tracking = evaluateTrackingCosts(context);
  score.path_lateral = tracking.path_lateral;
  score.heading = tracking.heading;
  score.progress = tracking.progress;
  score.terminal = evaluateTerminalCost(context);
  score.smooth = evaluateSmoothnessCost(context);
  score.speed = evaluateSpeedCost(context);
  return weightedTotal(score);
}

void ScoreAggregator::accumulateScore(ScoreBreakdown& total, const ScoreBreakdown& inc) const
{
  total.obstacle += inc.obstacle;
  total.path_lateral += inc.path_lateral;
  total.heading += inc.heading;
  total.progress += inc.progress;
  total.terminal += inc.terminal;
  total.smooth += inc.smooth;
  total.speed += inc.speed;
}

double ScoreAggregator::weightedTotal(const ScoreBreakdown& score) const
{
  return score.obstacle + score.path_lateral + score.heading + score.progress + score.terminal + score.smooth + score.speed;
}
}  // namespace lt_dwa_v2_adapter
