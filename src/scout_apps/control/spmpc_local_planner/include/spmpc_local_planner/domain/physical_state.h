#pragma once

#include <cmath>

namespace spmpc_local_planner {
namespace mainline {

// Shared physical state vocabulary.  These types deliberately contain no
// solver, command-history, release-grid, or virtual-progress semantics.
struct PlanarPoseState {
  double x{0.0};
  double y{0.0};
  double heading{0.0};
};

struct ActualMotionState {
  double linear_velocity{0.0};
  double angular_velocity{0.0};
};

struct LiquidModalState {
  double eta_x{0.0};
  double eta_x_dot{0.0};
  double eta_y{0.0};
  double eta_y_dot{0.0};
};

// Physical state only.  Virtual progress is deliberately absent because a
// known-prefix propagation has no authoritative historical progress control.
struct PhysicalPlantState {
  PlanarPoseState pose;
  ActualMotionState actual;
  LiquidModalState liquid;
};

inline bool isFinitePhysicalPlantState(
    const PhysicalPlantState& state) noexcept {
  return std::isfinite(state.pose.x) && std::isfinite(state.pose.y) &&
         std::isfinite(state.pose.heading) &&
         std::isfinite(state.actual.linear_velocity) &&
         std::isfinite(state.actual.angular_velocity) &&
         std::isfinite(state.liquid.eta_x) &&
         std::isfinite(state.liquid.eta_x_dot) &&
         std::isfinite(state.liquid.eta_y) &&
         std::isfinite(state.liquid.eta_y_dot);
}

}  // namespace mainline
}  // namespace spmpc_local_planner
