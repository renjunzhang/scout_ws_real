#pragma once

#include <nav_msgs/OccupancyGrid.h>

#include "lt_dwa_v2_adapter/core/planner_config.h"
#include "lt_dwa_v2_adapter/core/trajectory_types.h"
#include "lt_dwa_v2_adapter/geometry/planning_frames.h"

namespace lt_dwa_v2_adapter
{
class OccupancyAdapter
{
public:
  OccupancyAdapter(const nav_msgs::OccupancyGrid* grid,
                   const PlanningTransform2D& plan_to_map,
                   const OccupancyConfig& config);

  bool hasGrid() const;
  bool transformOk() const;
  bool collisionAt(const RobotState& state, CollisionDiagnostics* diagnostics = nullptr) const;
  double obstacleCost(const RobotState& state) const;

private:
  RobotState toMapState(const RobotState& state) const;
  bool worldToMap(double wx, double wy, int& mx, int& my) const;
  int occupancyAt(int mx, int my) const;

  const nav_msgs::OccupancyGrid* grid_ = nullptr;
  PlanningTransform2D plan_to_map_;
  OccupancyConfig config_;
};
}  // namespace lt_dwa_v2_adapter
