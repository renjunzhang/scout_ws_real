#pragma once

#include <nav_msgs/OccupancyGrid.h>

#include "lt_dwa_adapter/planning_frames.h"
#include "lt_dwa_adapter/trajectory_types.h"

namespace lt_dwa_adapter
{
struct OccupancyQueryConfig
{
  double robot_radius_m = 0.35;
  double clearance_radius_m = 0.80;
  int lethal_occupancy = 65;
  bool treat_unknown_as_occupied = false;
};

class OccupancyAdapter
{
public:
  OccupancyAdapter(const nav_msgs::OccupancyGrid* grid,
                   const PlanningTransform2D& plan_to_map,
                   const OccupancyQueryConfig& config);

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
  OccupancyQueryConfig config_;
};
}  // namespace lt_dwa_adapter
