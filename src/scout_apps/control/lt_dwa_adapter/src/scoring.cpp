#include "lt_dwa_adapter/lt_dwa_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

bool worldToMap(const nav_msgs::OccupancyGrid& grid, double wx, double wy, int& mx, int& my)
{
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const double res = grid.info.resolution;
  if (res <= 0.0)
    return false;
  mx = static_cast<int>(std::floor((wx - origin_x) / res));
  my = static_cast<int>(std::floor((wy - origin_y) / res));
  return mx >= 0 && my >= 0 && mx < static_cast<int>(grid.info.width) && my < static_cast<int>(grid.info.height);
}

int occupancyAt(const nav_msgs::OccupancyGrid& grid, int mx, int my)
{
  if (mx < 0 || my < 0 || mx >= static_cast<int>(grid.info.width) || my >= static_cast<int>(grid.info.height))
    return -1;
  const auto idx = static_cast<size_t>(my) * static_cast<size_t>(grid.info.width) + static_cast<size_t>(mx);
  if (idx >= grid.data.size())
    return -1;
  return static_cast<int>(grid.data[idx]);
}
}  // namespace

LtDwaPlanner::PathMatch LtDwaPlanner::matchPath(const RobotState& state, const std::vector<Pose2D>& path) const
{
  PathMatch best;
  best.distance = std::numeric_limits<double>::infinity();
  if (path.empty())
    return best;

  for (size_t i = 0; i < path.size(); ++i)
  {
    const double dx = state.x - path[i].x;
    const double dy = state.y - path[i].y;
    const double dist = std::hypot(dx, dy);
    if (dist < best.distance)
    {
      best.distance = dist;
      best.index = static_cast<double>(i);
      best.pose = path[i];
    }
  }
  best.heading_error = std::abs(normalizeAngle(state.yaw - best.pose.yaw));
  return best;
}

bool LtDwaPlanner::isGoalReached(const RobotState& state, const std::vector<Pose2D>& path) const
{
  if (path.empty())
    return false;
  const Pose2D& goal = path.back();
  const double dist = std::hypot(goal.x - state.x, goal.y - state.y);
  const double yaw_err = std::abs(normalizeAngle(goal.yaw - state.yaw));
  return dist <= config_.goal_xy_tolerance_m && yaw_err <= config_.goal_yaw_tolerance_rad;
}

bool LtDwaPlanner::collisionAt(const RobotState& state, const nav_msgs::OccupancyGrid* occupancy) const
{
  if (!occupancy || occupancy->data.empty())
    return false;

  const double step = std::max(static_cast<double>(occupancy->info.resolution), config_.robot_radius_m / 2.0);
  const int rings = std::max(1, static_cast<int>(std::ceil(config_.robot_radius_m / std::max(0.01, step))));
  for (int r = 0; r <= rings; ++r)
  {
    const double radius = (static_cast<double>(r) / static_cast<double>(rings)) * config_.robot_radius_m;
    const int samples = (r == 0) ? 1 : 12;
    for (int i = 0; i < samples; ++i)
    {
      const double angle = (samples == 1) ? 0.0 : (2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples));
      const double wx = state.x + radius * std::cos(angle);
      const double wy = state.y + radius * std::sin(angle);
      int mx = 0;
      int my = 0;
      if (!worldToMap(*occupancy, wx, wy, mx, my))
        return config_.treat_unknown_as_occupied;
      const int occ = occupancyAt(*occupancy, mx, my);
      if (occ < 0)
      {
        if (config_.treat_unknown_as_occupied)
          return true;
      }
      else if (occ >= config_.lethal_occupancy)
      {
        return true;
      }
    }
  }
  return false;
}

double LtDwaPlanner::obstacleCost(const RobotState& state, const nav_msgs::OccupancyGrid* occupancy) const
{
  if (!occupancy || occupancy->data.empty() || occupancy->info.resolution <= 0.0)
    return 0.0;

  int cx = 0;
  int cy = 0;
  if (!worldToMap(*occupancy, state.x, state.y, cx, cy))
    return config_.treat_unknown_as_occupied ? 1.0 : 0.2;

  const int cells = std::max(1, static_cast<int>(std::ceil(config_.clearance_radius_m / occupancy->info.resolution)));
  double min_dist = std::numeric_limits<double>::infinity();
  double occ_peak = 0.0;
  for (int dy = -cells; dy <= cells; ++dy)
  {
    for (int dx = -cells; dx <= cells; ++dx)
    {
      const int mx = cx + dx;
      const int my = cy + dy;
      const int occ = occupancyAt(*occupancy, mx, my);
      const double dist = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * occupancy->info.resolution;
      if (dist > config_.clearance_radius_m)
        continue;
      if (occ < 0)
      {
        if (config_.treat_unknown_as_occupied)
          min_dist = std::min(min_dist, dist);
        continue;
      }
      occ_peak = std::max(occ_peak, static_cast<double>(occ) / 100.0);
      if (occ >= config_.lethal_occupancy)
        min_dist = std::min(min_dist, dist);
    }
  }

  if (!std::isfinite(min_dist))
    return occ_peak;
  const double clearance = std::max(0.0, min_dist - config_.robot_radius_m);
  const double clearance_cost = 1.0 / (1.0 + clearance);
  return std::max(occ_peak, clearance_cost);
}

double LtDwaPlanner::scorePoint(const RobotState& state,
                                const Command& command,
                                const Command& previous_command,
                                double previous_progress_index,
                                const std::vector<Pose2D>& path,
                                const nav_msgs::OccupancyGrid* occupancy,
                                ScoreBreakdown& score) const
{
  const PathMatch match = matchPath(state, path);
  const Pose2D& goal = path.back();
  const double terminal_dist = std::hypot(goal.x - state.x, goal.y - state.y);
  const double path_scale = std::max(1.0, static_cast<double>(path.size() - 1));
  const double progress_delta = match.index - previous_progress_index;
  const double v_fraction = config_.limits.v_max_mps > 1e-9 ? command.v / config_.limits.v_max_mps : 0.0;
  const double goal_slowdown = std::max(0.0, std::min(1.0, terminal_dist / 0.80));

  score.obstacle = config_.weights.obstacle * obstacleCost(state, occupancy);
  score.path_lateral = config_.weights.path_lateral * match.distance;
  score.heading = config_.weights.heading * match.heading_error;
  score.progress = -config_.weights.progress * std::max(0.0, progress_delta) / path_scale;
  if (progress_delta < -0.25)
    score.progress += config_.weights.progress * std::abs(progress_delta) / path_scale;
  score.terminal = config_.weights.terminal * terminal_dist;
  score.smooth = config_.weights.smooth_v * std::abs(command.v - previous_command.v) +
                 config_.weights.smooth_omega * std::abs(command.omega - previous_command.omega) +
                 0.25 * config_.weights.smooth_omega * std::abs(command.omega);
  score.speed = -config_.weights.speed * std::max(0.0, v_fraction) * goal_slowdown;
  if (terminal_dist > config_.goal_xy_tolerance_m && command.v < 0.03)
    score.speed += 0.5 * config_.weights.speed;
  if (terminal_dist < 0.80)
    score.speed += 4.0 * config_.weights.speed * (1.0 - goal_slowdown) * std::max(0.0, v_fraction);
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
