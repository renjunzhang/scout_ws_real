#include "lt_dwa_adapter/lt_dwa_planner.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace lt_dwa_adapter
{
namespace
{
double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(hi, value));
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

Command LtDwaPlanner::pathTrackingSeed(const RobotState& state,
                                       const PathReference& path,
                                       double previous_progress_s) const
{
  Command seed;
  if (path.empty())
    return seed;

  const double min_progress_s = std::max(0.0, previous_progress_s - config_.progress_rollback_tolerance_m);
  const double max_progress_s = previous_progress_s + config_.max_progress_advance_per_step_m;
  const PathProjection match = path.project(state, min_progress_s, max_progress_s);
  if (!match.valid)
    return seed;

  const Pose2D target = path.sampleByProgress(match.progress_s + config_.lookahead_distance_m);
  const Pose2D goal = path.sampleByProgress(path.totalLength());
  const double terminal_dist = std::hypot(goal.x - state.x, goal.y - state.y);
  const double target_heading = std::atan2(target.y - state.y, target.x - state.x);
  const double pure_pursuit_error = normalizeAngle(target_heading - state.yaw);
  const double lateral_correction = std::atan2(-config_.cross_track_heading_gain * match.signed_lateral_error,
                                               std::max(0.20, config_.lookahead_distance_m));
  const double corrected_path_heading = normalizeAngle(match.pose.yaw + lateral_correction);
  const double cross_track_error = normalizeAngle(corrected_path_heading - state.yaw);
  const double heading_error = normalizeAngle(0.45 * pure_pursuit_error + 0.55 * cross_track_error);
  const double tracking_heading_error = std::max(std::abs(pure_pursuit_error), std::abs(cross_track_error));
  const double yaw_gain = match.distance > config_.tracking_slowdown_lateral_m ? 2.2 : (terminal_dist < 0.45 ? 1.2 : 1.7);
  const double desired_omega = clamp(yaw_gain * heading_error, -config_.limits.omega_max_radps, config_.limits.omega_max_radps);
  const double heading_speed_scale = clamp(std::cos(std::min(M_PI / 2.0, tracking_heading_error)), 0.08, 1.0);
  const double lateral_speed_scale = clamp(1.0 - match.distance / std::max(0.10, config_.max_tracking_deviation_m), 0.10, 1.0);
  const double approach_scale = clamp(terminal_dist / 0.75, 0.15, 1.0);
  const double desired_v = config_.limits.v_max_mps * heading_speed_scale * lateral_speed_scale * approach_scale;

  const double v_step = config_.limits.a_max_mps2 * config_.dt;
  const double w_step = config_.limits.alpha_max_radps2 * config_.dt;
  const double min_v = config_.limits.allow_reverse ? -config_.limits.v_max_mps : 0.0;
  seed.v = clamp(desired_v, clamp(state.v - v_step, min_v, config_.limits.v_max_mps),
                 clamp(state.v + v_step, min_v, config_.limits.v_max_mps));
  seed.omega = clamp(desired_omega,
                     clamp(state.omega - w_step, -config_.limits.omega_max_radps, config_.limits.omega_max_radps),
                     clamp(state.omega + w_step, -config_.limits.omega_max_radps, config_.limits.omega_max_radps));
  return seed;
}

PlanResult LtDwaPlanner::plan(const RobotState& current,
                              const std::vector<Pose2D>& path_points,
                              const OccupancyAdapter* occupancy,
                              double min_progress_s,
                              double max_progress_s) const
{
  PlanResult result;
  PathReference path;
  if (!path.setPath(path_points))
  {
    result.status = "WAITING_FOR_PATH";
    return result;
  }
  result.diagnostics.plan_map_transform_ok = !occupancy || occupancy->transformOk();

  if (max_progress_s < 0.0)
    max_progress_s = path.totalLength();
  max_progress_s = clamp(max_progress_s, min_progress_s, path.totalLength());
  const PathProjection initial_match = path.project(current, min_progress_s, max_progress_s);
  result.diagnostics.has_initial_match = initial_match.valid;
  if (initial_match.valid)
  {
    result.diagnostics.initial_match_index = initial_match.index;
    result.diagnostics.initial_match_distance = initial_match.distance;
    result.diagnostics.initial_signed_lateral_error = initial_match.signed_lateral_error;
    result.diagnostics.initial_match_heading_error = initial_match.heading_error;
    result.diagnostics.initial_progress_s = initial_match.progress_s;
    const double target_progress_s = std::min(path.totalLength(), initial_match.progress_s + config_.lookahead_distance_m);
    const Pose2D target = path.sampleByProgress(target_progress_s);
    result.diagnostics.lookahead_target_index = static_cast<int>(std::round(initial_match.index));
    result.diagnostics.lookahead_target_x = target.x;
    result.diagnostics.lookahead_target_y = target.y;
    result.diagnostics.lookahead_target_progress_s = target_progress_s;
  }
  result.diagnostics.max_tracking_deviation_m = config_.max_tracking_deviation_m;

  if (isGoalReached(current, path))
  {
    result.valid = true;
    result.status = "GOAL_REACHED";
    return result;
  }

  CollisionDiagnostics initial_collision_details;
  const bool initial_collision = collisionAt(current, occupancy, &initial_collision_details);
  result.diagnostics.initial_collision = initial_collision;
  result.diagnostics.initial_collision_details = initial_collision_details;
  if (initial_collision)
  {
    result.status = "ROBOT_IN_COLLISION";
    return result;
  }
  if (config_.max_tracking_deviation_m > 0.0 && initial_match.valid &&
      initial_match.distance > config_.max_tracking_deviation_m)
  {
    result.diagnostics.tracking_diverged = true;
    result.status = "TRACKING_DIVERGED";
    return result;
  }

  TrajectoryCandidate root;
  root.valid = true;
  root.total_cost = 0.0;
  root.progress_index = initial_match.index;
  root.progress_s = initial_match.progress_s;
  std::vector<TrajectoryCandidate> frontier;
  frontier.push_back(root);

  const Command initial_command{ current.v, current.omega };

  for (int layer = 0; layer < config_.horizon_steps; ++layer)
  {
    std::vector<TrajectoryCandidate> next_frontier;
    for (const auto& candidate : frontier)
    {
      const RobotState base_state = candidate.points.empty() ? current : candidate.points.back().state;
      const Command previous_command = candidate.points.empty() ? initial_command : candidate.points.back().command;
      auto commands = sampleCommands(base_state);
      commands.push_back(pathTrackingSeed(base_state, path, candidate.progress_s));
      for (const auto& command : commands)
      {
        ++result.expanded_nodes;
        const RobotState next_state = rolloutStep(base_state, command);
        if (collisionAt(next_state, occupancy))
          continue;

        ScoreBreakdown inc_score;
        const double inc_cost = scorePoint(next_state, command, previous_command, candidate.progress_s, path, occupancy, inc_score);
        const double min_progress_s = std::max(0.0, candidate.progress_s - config_.progress_rollback_tolerance_m);
        const double max_progress_s = candidate.progress_s + config_.max_progress_advance_per_step_m;
        const PathProjection next_match = path.project(next_state, min_progress_s, max_progress_s);
        if (!next_match.valid)
          continue;

        TrajectoryCandidate child = candidate;
        TrajectoryPoint point;
        point.state = next_state;
        point.command = command;
        point.score = inc_score;
        point.incremental_cost = inc_cost;
        child.points.push_back(point);
        if (candidate.points.empty())
          child.first_command = command;
        child.total_cost += inc_cost;
        accumulateScore(child.score, inc_score);
        child.progress_index = next_match.index;
        child.progress_s = next_match.progress_s;
        child.valid = true;
        next_frontier.push_back(child);
      }
    }

    if (next_frontier.empty())
      break;

    std::sort(next_frontier.begin(), next_frontier.end(), [](const TrajectoryCandidate& a, const TrajectoryCandidate& b) {
      if (std::abs(a.total_cost - b.total_cost) > 1e-9)
        return a.total_cost < b.total_cost;
      return a.progress_s > b.progress_s;
    });
    if (static_cast<int>(next_frontier.size()) > config_.top_k_per_layer)
      next_frontier.resize(static_cast<size_t>(config_.top_k_per_layer));
    frontier.swap(next_frontier);
  }

  result.valid_candidates = static_cast<int>(frontier.size());
  if (frontier.empty())
  {
    result.status = "NO_VALID_CMD";
    return result;
  }

  const auto best_it = std::min_element(frontier.begin(), frontier.end(), [](const TrajectoryCandidate& a, const TrajectoryCandidate& b) {
    if (std::abs(a.total_cost - b.total_cost) > 1e-9)
      return a.total_cost < b.total_cost;
    return a.progress_s > b.progress_s;
  });
  result.best = *best_it;
  result.command = result.best.first_command;
  result.valid = true;
  result.status = "TRACKING";
  return result;
}

std::string formatStatus(const PlanResult& result)
{
  std::ostringstream ss;
  ss << result.status;
  ss << " expanded=" << result.expanded_nodes;
  ss << " valid=" << result.valid_candidates;
  if (result.valid && result.status == "TRACKING")
  {
    ss << " cmd_v=" << result.command.v;
    ss << " cmd_w=" << result.command.omega;
    ss << " cost=" << result.best.total_cost;
  }
  return ss.str();
}
}  // namespace lt_dwa_adapter
