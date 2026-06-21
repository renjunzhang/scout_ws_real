#include "lt_dwa_adapter/occupancy_adapter.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lt_dwa_adapter
{
OccupancyAdapter::OccupancyAdapter(const nav_msgs::OccupancyGrid* grid,
                                   const PlanningTransform2D& plan_to_map,
                                   const OccupancyQueryConfig& config)
  : grid_(grid)
  , plan_to_map_(plan_to_map)
  , config_(config)
{
}

bool OccupancyAdapter::hasGrid() const
{
  return grid_ && !grid_->data.empty();
}

bool OccupancyAdapter::transformOk() const
{
  return plan_to_map_.valid;
}

RobotState OccupancyAdapter::toMapState(const RobotState& state) const
{
  return transformState(state, plan_to_map_);
}

bool OccupancyAdapter::worldToMap(double wx, double wy, int& mx, int& my) const
{
  if (!hasGrid())
    return false;
  const double origin_x = grid_->info.origin.position.x;
  const double origin_y = grid_->info.origin.position.y;
  const double res = grid_->info.resolution;
  if (res <= 0.0)
    return false;
  mx = static_cast<int>(std::floor((wx - origin_x) / res));
  my = static_cast<int>(std::floor((wy - origin_y) / res));
  return mx >= 0 && my >= 0 && mx < static_cast<int>(grid_->info.width) && my < static_cast<int>(grid_->info.height);
}

int OccupancyAdapter::occupancyAt(int mx, int my) const
{
  if (!hasGrid() || mx < 0 || my < 0 || mx >= static_cast<int>(grid_->info.width) ||
      my >= static_cast<int>(grid_->info.height))
  {
    return -1;
  }
  const auto idx = static_cast<size_t>(my) * static_cast<size_t>(grid_->info.width) + static_cast<size_t>(mx);
  if (idx >= grid_->data.size())
    return -1;
  return static_cast<int>(grid_->data[idx]);
}

bool OccupancyAdapter::collisionAt(const RobotState& state, CollisionDiagnostics* diagnostics) const
{
  if (diagnostics)
    *diagnostics = CollisionDiagnostics{};
  if (!hasGrid())
    return false;
  if (!plan_to_map_.valid)
    return true;

  bool colliding = false;
  const RobotState map_state = toMapState(state);
  const double step = std::max(static_cast<double>(grid_->info.resolution), config_.robot_radius_m / 2.0);
  const int rings = std::max(1, static_cast<int>(std::ceil(config_.robot_radius_m / std::max(0.01, step))));
  for (int r = 0; r <= rings; ++r)
  {
    const double radius = (static_cast<double>(r) / static_cast<double>(rings)) * config_.robot_radius_m;
    const int samples = (r == 0) ? 1 : 12;
    for (int i = 0; i < samples; ++i)
    {
      const double angle = (samples == 1) ? 0.0 : (2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples));
      const double wx = map_state.x + radius * std::cos(angle);
      const double wy = map_state.y + radius * std::sin(angle);
      if (diagnostics)
        diagnostics->checked_samples += 1;
      int mx = 0;
      int my = 0;
      if (!worldToMap(wx, wy, mx, my))
      {
        if (diagnostics)
          diagnostics->out_of_map_samples += 1;
        if (config_.treat_unknown_as_occupied)
        {
          colliding = true;
          if (!diagnostics)
            return true;
        }
        continue;
      }
      const int occ = occupancyAt(mx, my);
      if (diagnostics && r == 0 && i == 0)
        diagnostics->center_occupancy = occ;
      if (diagnostics && occ >= 0)
        diagnostics->max_occupancy = std::max(diagnostics->max_occupancy, occ);
      if (occ < 0)
      {
        if (diagnostics)
          diagnostics->unknown_samples += 1;
        if (config_.treat_unknown_as_occupied)
        {
          colliding = true;
          if (!diagnostics)
            return true;
        }
      }
      else if (occ >= config_.lethal_occupancy)
      {
        colliding = true;
        if (diagnostics)
        {
          diagnostics->lethal_samples += 1;
          if (!diagnostics->has_first_lethal_sample)
          {
            diagnostics->has_first_lethal_sample = true;
            diagnostics->first_lethal_x = wx;
            diagnostics->first_lethal_y = wy;
          }
        }
        else
        {
          return true;
        }
      }
    }
  }
  return colliding;
}

double OccupancyAdapter::obstacleCost(const RobotState& state) const
{
  if (!hasGrid() || grid_->info.resolution <= 0.0)
    return 0.0;
  if (!plan_to_map_.valid)
    return config_.treat_unknown_as_occupied ? 1.0 : 0.2;

  const RobotState map_state = toMapState(state);
  int cx = 0;
  int cy = 0;
  if (!worldToMap(map_state.x, map_state.y, cx, cy))
    return config_.treat_unknown_as_occupied ? 1.0 : 0.2;

  const int cells = std::max(1, static_cast<int>(std::ceil(config_.clearance_radius_m / grid_->info.resolution)));
  double min_dist = std::numeric_limits<double>::infinity();
  double occ_peak = 0.0;
  for (int dy = -cells; dy <= cells; ++dy)
  {
    for (int dx = -cells; dx <= cells; ++dx)
    {
      const int mx = cx + dx;
      const int my = cy + dy;
      const int occ = occupancyAt(mx, my);
      const double dist = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * grid_->info.resolution;
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
}  // namespace lt_dwa_adapter
