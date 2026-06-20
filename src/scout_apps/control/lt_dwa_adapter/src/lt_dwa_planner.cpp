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

Command LtDwaPlanner::pathTrackingSeed(const RobotState& state, const std::vector<Pose2D>& path) const
{
  Command seed;
  if (path.empty())
    return seed;

  const PathMatch match = matchPath(state, path);
  const double lookahead_m = 0.55;
  size_t target_index = static_cast<size_t>(std::max(0.0, match.index));
  double accumulated = 0.0;
  for (size_t i = target_index; i + 1 < path.size(); ++i)
  {
    const double ds = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    accumulated += ds;
    target_index = i + 1;
    if (accumulated >= lookahead_m)
      break;
  }

  const Pose2D& target = path[target_index];
  const Pose2D& goal = path.back();
  const double terminal_dist = std::hypot(goal.x - state.x, goal.y - state.y);
  const double target_heading = std::atan2(target.y - state.y, target.x - state.x);
  const double heading_error = normalizeAngle(target_heading - state.yaw);
  const double yaw_gain = terminal_dist < 0.45 ? 1.2 : 1.6;
  const double desired_omega = clamp(yaw_gain * heading_error, -config_.limits.omega_max_radps, config_.limits.omega_max_radps);
  const double speed_scale = clamp(std::cos(std::min(M_PI / 2.0, std::abs(heading_error))), 0.20, 1.0);
  const double approach_scale = clamp(terminal_dist / 0.75, 0.15, 1.0);
  const double desired_v = config_.limits.v_max_mps * speed_scale * approach_scale;

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
                              const std::vector<Pose2D>& path,
                              const nav_msgs::OccupancyGrid* occupancy) const
{
  PlanResult result;
  if (path.empty())
  {
    result.status = "WAITING_FOR_PATH";
    return result;
  }
  if (isGoalReached(current, path))
  {
    result.valid = true;
    result.status = "GOAL_REACHED";
    return result;
  }
  if (collisionAt(current, occupancy))
  {
    result.status = "ROBOT_IN_COLLISION";
    return result;
  }

  TrajectoryCandidate root;
  root.valid = true;
  root.total_cost = 0.0;
  root.progress_index = matchPath(current, path).index;
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
      commands.push_back(pathTrackingSeed(base_state, path));
      for (const auto& command : commands)
      {
        ++result.expanded_nodes;
        const RobotState next_state = rolloutStep(base_state, command);
        if (collisionAt(next_state, occupancy))
          continue;

        ScoreBreakdown inc_score;
        const double inc_cost = scorePoint(next_state, command, previous_command, candidate.progress_index, path, occupancy, inc_score);
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
        child.progress_index = matchPath(next_state, path).index;
        child.valid = true;
        next_frontier.push_back(child);
      }
    }

    if (next_frontier.empty())
      break;

    std::sort(next_frontier.begin(), next_frontier.end(), [](const TrajectoryCandidate& a, const TrajectoryCandidate& b) {
      if (std::abs(a.total_cost - b.total_cost) > 1e-9)
        return a.total_cost < b.total_cost;
      return a.progress_index > b.progress_index;
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
    return a.progress_index > b.progress_index;
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
