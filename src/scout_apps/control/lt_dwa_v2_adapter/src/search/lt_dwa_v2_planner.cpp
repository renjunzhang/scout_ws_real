#include "lt_dwa_v2_adapter/search/lt_dwa_v2_planner.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "lt_dwa_v2_adapter/search/candidate_lattice.h"

namespace lt_dwa_v2_adapter
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

LtDwaV2Planner::LtDwaV2Planner()
{
  configure(PlannerConfig{});
}

LtDwaV2Planner::LtDwaV2Planner(const PlannerConfig& config)
{
  configure(config);
}

void LtDwaV2Planner::configure(const PlannerConfig& config)
{
  config_ = config;
  config_.rollout.dt = std::max(0.02, config_.rollout.dt);
  config_.rollout.horizon_steps = std::max(1, config_.rollout.horizon_steps);
  config_.sampling.v_samples = std::max(1, config_.sampling.v_samples);
  config_.sampling.omega_samples = std::max(1, config_.sampling.omega_samples);
  config_.search.top_k_per_layer = std::max(1, config_.search.top_k_per_layer);
  config_.limits.v_max_mps = std::max(0.0, config_.limits.v_max_mps);
  config_.limits.omega_max_radps = std::max(0.0, config_.limits.omega_max_radps);
  config_.limits.a_max_mps2 = std::max(0.0, config_.limits.a_max_mps2);
  config_.limits.alpha_max_radps2 = std::max(0.0, config_.limits.alpha_max_radps2);
  config_.tracking.lookahead_distance_m = std::max(0.05, config_.tracking.lookahead_distance_m);
  config_.tracking.progress_rollback_tolerance_m = std::max(0.0, config_.tracking.progress_rollback_tolerance_m);
  config_.tracking.max_progress_advance_per_step_m = std::max(0.05, config_.tracking.max_progress_advance_per_step_m);
  config_.tracking.cross_track_heading_gain = std::max(0.0, config_.tracking.cross_track_heading_gain);
  config_.tracking.tracking_slowdown_lateral_m = std::max(0.05, config_.tracking.tracking_slowdown_lateral_m);
  config_.tracking.tracking_slowdown_heading_rad = std::max(0.05, config_.tracking.tracking_slowdown_heading_rad);

  sampler_.configure(config_.limits, config_.sampling, config_.rollout);
  rollout_.configure(config_.rollout);
  scorer_.configure(config_);
  pruner_.configure(config_.search);
}

Command LtDwaV2Planner::pathTrackingSeed(const RobotState& state,
                                         const PathReference& path,
                                         double previous_progress_s) const
{
  Command seed;
  if (path.empty())
    return seed;

  const double min_progress_s = std::max(0.0, previous_progress_s);
  const double max_progress_s = previous_progress_s + config_.tracking.max_progress_advance_per_step_m;
  const PathProjection match = path.project(state, min_progress_s, max_progress_s);
  if (!match.valid)
    return seed;

  const Pose2D target = path.sampleByProgress(match.progress_s + config_.tracking.lookahead_distance_m);
  const Pose2D goal = path.sampleByProgress(path.totalLength());
  const double terminal_dist = std::hypot(goal.x - state.x, goal.y - state.y);
  const double target_heading = std::atan2(target.y - state.y, target.x - state.x);
  const double pure_pursuit_error = normalizeAngle(target_heading - state.yaw);
  const double lateral_correction = std::atan2(-config_.tracking.cross_track_heading_gain * match.signed_lateral_error,
                                               std::max(0.20, config_.tracking.lookahead_distance_m));
  const double corrected_path_heading = normalizeAngle(match.pose.yaw + lateral_correction);
  const double cross_track_error = normalizeAngle(corrected_path_heading - state.yaw);
  const double terminal_blend = terminal_dist < 1.0 ? 0.75 : 0.20;
  const double heading_error = normalizeAngle(terminal_blend * pure_pursuit_error +
                                             (1.0 - terminal_blend) * cross_track_error);
  const double tracking_heading_error = std::max(std::abs(pure_pursuit_error), std::abs(cross_track_error));
  const double yaw_gain = match.distance > config_.tracking.tracking_slowdown_lateral_m ?
                              2.2 :
                              (terminal_dist < 0.45 ? 1.2 : 1.7);
  const double desired_omega =
      clamp(yaw_gain * heading_error, -config_.limits.omega_max_radps, config_.limits.omega_max_radps);
  const double heading_speed_scale = clamp(std::cos(std::min(M_PI / 2.0, tracking_heading_error)), 0.08, 1.0);
  const double lateral_speed_scale =
      clamp(1.0 - match.distance / std::max(0.10, config_.tracking.max_tracking_deviation_m), 0.10, 1.0);
  const double approach_scale = clamp(terminal_dist / 0.75, 0.15, 1.0);
  double desired_v = config_.limits.v_max_mps * heading_speed_scale * lateral_speed_scale * approach_scale;
  if (terminal_dist > config_.goal.xy_tolerance_m && tracking_heading_error < 1.20)
    desired_v = std::max(desired_v, 0.12 * config_.limits.v_max_mps);

  const DynamicWindow window = sampler_.windowFor(state);
  seed.v = clamp(desired_v, window.min_v, window.max_v);
  seed.omega = clamp(desired_omega, window.min_omega, window.max_omega);
  return seed;
}

PlanResult LtDwaV2Planner::plan(const RobotState& current,
                                const std::vector<Pose2D>& path_points,
                                const OccupancyAdapter* occupancy,
                                double min_progress_s,
                                double max_progress_s) const
{
  PlanResult result;
  PathReference path;
  if (!path.setPath(path_points))
  {
    setStatus(result, PlannerStatusCode::WaitingForPath);
    return result;
  }
  result.diagnostics.plan_map_transform_ok = !occupancy || occupancy->transformOk();

  min_progress_s = clamp(min_progress_s, 0.0, path.totalLength());
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
    const double target_progress_s =
        std::min(path.totalLength(), initial_match.progress_s + config_.tracking.lookahead_distance_m);
    const Pose2D target = path.sampleByProgress(target_progress_s);
    result.diagnostics.lookahead_target_index = static_cast<int>(std::round(initial_match.index));
    result.diagnostics.lookahead_target_x = target.x;
    result.diagnostics.lookahead_target_y = target.y;
    result.diagnostics.lookahead_target_progress_s = target_progress_s;
  }
  result.diagnostics.max_tracking_deviation_m = config_.tracking.max_tracking_deviation_m;

  if (isGoalReached(current, path))
  {
    result.valid = true;
    setStatus(result, PlannerStatusCode::GoalReached);
    return result;
  }

  CollisionDiagnostics initial_collision_details;
  const bool initial_collision = collisionAt(current, occupancy, &initial_collision_details);
  result.diagnostics.initial_collision = initial_collision;
  result.diagnostics.initial_collision_details = initial_collision_details;
  if (initial_collision)
  {
    setStatus(result, PlannerStatusCode::RobotInCollision);
    return result;
  }
  if (config_.tracking.max_tracking_deviation_m > 0.0 && initial_match.valid &&
      initial_match.distance > config_.tracking.max_tracking_deviation_m)
  {
    result.diagnostics.tracking_diverged = true;
    setStatus(result, PlannerStatusCode::TrackingDiverged);
    return result;
  }
  if (!initial_match.valid)
  {
    setStatus(result, PlannerStatusCode::NoValidCommand);
    return result;
  }

  CandidateFrontier frontier;
  frontier.push_back(makeRootCandidate(initial_match));
  const Command initial_command{ current.v, current.omega };

  for (int layer = 0; layer < config_.rollout.horizon_steps; ++layer)
  {
    CandidateFrontier next_frontier;
    for (const auto& candidate : frontier)
    {
      const RobotState base_state = candidate.points.empty() ? current : candidate.points.back().state;
      const Command previous_command = candidate.points.empty() ? initial_command : candidate.points.back().command;
      auto commands = sampler_.sampleCommands(base_state);
      commands.push_back(pathTrackingSeed(base_state, path, candidate.progress_s));
      for (const auto& command : commands)
      {
        ++result.expanded_nodes;
        const RobotState next_state = rollout_.step(base_state, command);
        if (collisionAt(next_state, occupancy))
          continue;

        ScoreBreakdown inc_score;
        const double inc_cost = scorer_.scorePoint(next_state, command, previous_command, candidate.progress_s,
                                                   path, occupancy, inc_score);
        const double next_min_progress_s =
            std::max(0.0, candidate.progress_s - config_.tracking.progress_rollback_tolerance_m);
        const double next_max_progress_s = candidate.progress_s + config_.tracking.max_progress_advance_per_step_m;
        const PathProjection next_match = path.project(next_state, next_min_progress_s, next_max_progress_s);
        if (!next_match.valid)
          continue;

        TrajectoryPoint point;
        point.state = next_state;
        point.command = command;
        point.score = inc_score;
        point.incremental_cost = inc_cost;
        next_frontier.push_back(appendCandidatePoint(candidate, point, next_match));
      }
    }

    if (next_frontier.empty())
      break;
    pruner_.prune(next_frontier);
    frontier.swap(next_frontier);
  }

  result.valid_candidates = static_cast<int>(frontier.size());
  const TrajectoryCandidate* best = pruner_.best(frontier);
  if (!best)
  {
    setStatus(result, PlannerStatusCode::NoValidCommand);
    return result;
  }

  result.best = *best;
  result.command = result.best.first_command;
  result.valid = true;
  setStatus(result, PlannerStatusCode::Tracking);
  return result;
}

bool LtDwaV2Planner::isGoalReached(const RobotState& state, const PathReference& path) const
{
  if (path.empty())
    return false;
  const Pose2D goal = path.sampleByProgress(path.totalLength());
  const double dist = std::hypot(goal.x - state.x, goal.y - state.y);
  const double yaw_err = std::abs(normalizeAngle(goal.yaw - state.yaw));
  return dist <= config_.goal.xy_tolerance_m && yaw_err <= config_.goal.yaw_tolerance_rad;
}

bool LtDwaV2Planner::collisionAt(const RobotState& state,
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

void LtDwaV2Planner::setStatus(PlanResult& result, PlannerStatusCode status) const
{
  result.status_code = status;
  result.status = statusString(status);
}

std::string formatStatus(const PlanResult& result)
{
  std::ostringstream ss;
  ss << result.status;
  ss << " expanded=" << result.expanded_nodes;
  ss << " valid=" << result.valid_candidates;
  if (result.valid && result.status_code == PlannerStatusCode::Tracking)
  {
    ss << " cmd_v=" << result.command.v;
    ss << " cmd_w=" << result.command.omega;
    ss << " cost=" << result.best.total_cost;
  }
  return ss.str();
}
}  // namespace lt_dwa_v2_adapter
