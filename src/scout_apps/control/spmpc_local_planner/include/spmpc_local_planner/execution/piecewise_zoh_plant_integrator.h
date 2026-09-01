#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "spmpc_local_planner/execution/actuator_response_params.h"

namespace spmpc_local_planner {
namespace mainline {

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

struct LiquidModalParams {
  double natural_frequency_rad_per_sec{0.0};
  double damping_ratio{0.0};
  double longitudinal_coupling{0.0};
  double lateral_coupling{0.0};
};

struct ZohPlantParams {
  FopdtChannelParams linear_actuator;
  FopdtChannelParams angular_actuator;
  LiquidModalParams liquid;
};

struct ZohTargetSegment {
  double duration_sec{0.0};
  double linear_target{0.0};
  double angular_target{0.0};
};

enum class PlantPropagationStatus : std::uint8_t {
  kOk = 0,
  kInvalidState,
  kInvalidTarget,
  kInvalidDuration,
  kNonFiniteOutput,
};

inline bool isValidLiquidModalParams(
    const LiquidModalParams& params) noexcept {
  if (!std::isfinite(params.natural_frequency_rad_per_sec) ||
      params.natural_frequency_rad_per_sec <= 0.0 ||
      !std::isfinite(params.damping_ratio) || params.damping_ratio < 0.0 ||
      !std::isfinite(params.longitudinal_coupling) ||
      params.longitudinal_coupling <= 0.0 ||
      !std::isfinite(params.lateral_coupling) ||
      params.lateral_coupling <= 0.0) {
    return false;
  }
  const double damping = 2.0 * params.damping_ratio *
                         params.natural_frequency_rad_per_sec;
  const double stiffness = params.natural_frequency_rad_per_sec *
                           params.natural_frequency_rad_per_sec;
  return std::isfinite(damping) && std::isfinite(stiffness);
}

inline bool isValidZohPlantParams(const ZohPlantParams& params) noexcept {
  return isValidFopdtChannel(params.linear_actuator) &&
         isValidFopdtChannel(params.angular_actuator) &&
         isValidLiquidModalParams(params.liquid);
}

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

namespace piecewise_plant_detail {

using LiquidDerivative = std::array<double, 4>;

inline bool actualAtElapsed(
    const ActualMotionState& initial, const ZohTargetSegment& segment,
    const ZohPlantParams& params, double elapsed_sec,
    ActualMotionState& actual) noexcept {
  double linear = initial.linear_velocity;
  double angular = initial.angular_velocity;
  if (fopdtStep(initial.linear_velocity, segment.linear_target, elapsed_sec,
                params.linear_actuator, linear) != FopdtStepStatus::kOk ||
      fopdtStep(initial.angular_velocity, segment.angular_target, elapsed_sec,
                params.angular_actuator, angular) != FopdtStepStatus::kOk) {
    return false;
  }
  actual.linear_velocity = linear;
  actual.angular_velocity = angular;
  return true;
}

inline bool addScaled(const LiquidModalState& initial,
                      const LiquidDerivative& derivative, double scale,
                      LiquidModalState& result) noexcept {
  result.eta_x = initial.eta_x + scale * derivative[0];
  result.eta_x_dot = initial.eta_x_dot + scale * derivative[1];
  result.eta_y = initial.eta_y + scale * derivative[2];
  result.eta_y_dot = initial.eta_y_dot + scale * derivative[3];
  return std::isfinite(result.eta_x) &&
         std::isfinite(result.eta_x_dot) &&
         std::isfinite(result.eta_y) &&
         std::isfinite(result.eta_y_dot);
}

inline bool liquidDerivativeAt(
    const LiquidModalState& liquid, const ActualMotionState& initial_actual,
    const ZohTargetSegment& segment, const ZohPlantParams& params,
    double elapsed_sec, LiquidDerivative& derivative) noexcept {
  ActualMotionState actual;
  if (!actualAtElapsed(initial_actual, segment, params, elapsed_sec, actual)) {
    return false;
  }

  const double linear_steady_state =
      params.linear_actuator.gain * segment.linear_target;
  const double longitudinal_acceleration =
      (linear_steady_state - actual.linear_velocity) /
      params.linear_actuator.tau_sec;
  const double lateral_acceleration =
      actual.linear_velocity * actual.angular_velocity;
  const double damping = 2.0 * params.liquid.damping_ratio *
                         params.liquid.natural_frequency_rad_per_sec;
  const double stiffness = params.liquid.natural_frequency_rad_per_sec *
                           params.liquid.natural_frequency_rad_per_sec;

  derivative[0] = liquid.eta_x_dot;
  derivative[1] =
      -damping * liquid.eta_x_dot - stiffness * liquid.eta_x -
      params.liquid.longitudinal_coupling * longitudinal_acceleration;
  derivative[2] = liquid.eta_y_dot;
  derivative[3] =
      -damping * liquid.eta_y_dot - stiffness * liquid.eta_y -
      params.liquid.lateral_coupling * lateral_acceleration;
  for (const double value : derivative) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return std::isfinite(linear_steady_state) &&
         std::isfinite(longitudinal_acceleration) &&
         std::isfinite(lateral_acceleration);
}

inline bool rk4Liquid(const LiquidModalState& initial_liquid,
                      const ActualMotionState& initial_actual,
                      const ZohTargetSegment& segment,
                      const ZohPlantParams& params,
                      LiquidModalState& next_liquid) noexcept {
  if (segment.duration_sec == 0.0) {
    next_liquid = initial_liquid;
    return true;
  }

  const double half_duration = 0.5 * segment.duration_sec;
  LiquidDerivative k1{};
  LiquidDerivative k2{};
  LiquidDerivative k3{};
  LiquidDerivative k4{};
  LiquidModalState intermediate;
  if (!liquidDerivativeAt(initial_liquid, initial_actual, segment, params,
                          0.0, k1) ||
      !addScaled(initial_liquid, k1, half_duration, intermediate) ||
      !liquidDerivativeAt(intermediate, initial_actual, segment, params,
                          half_duration, k2) ||
      !addScaled(initial_liquid, k2, half_duration, intermediate) ||
      !liquidDerivativeAt(intermediate, initial_actual, segment, params,
                          half_duration, k3) ||
      !addScaled(initial_liquid, k3, segment.duration_sec, intermediate) ||
      !liquidDerivativeAt(intermediate, initial_actual, segment, params,
                          segment.duration_sec, k4)) {
    return false;
  }

  const double scale = segment.duration_sec / 6.0;
  next_liquid.eta_x = initial_liquid.eta_x +
                      scale * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
  next_liquid.eta_x_dot =
      initial_liquid.eta_x_dot +
      scale * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
  next_liquid.eta_y = initial_liquid.eta_y +
                      scale * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]);
  next_liquid.eta_y_dot =
      initial_liquid.eta_y_dot +
      scale * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]);
  return std::isfinite(next_liquid.eta_x) &&
         std::isfinite(next_liquid.eta_x_dot) &&
         std::isfinite(next_liquid.eta_y) &&
         std::isfinite(next_liquid.eta_y_dot);
}

}  // namespace piecewise_plant_detail

// Stateless with respect to command history.  A caller may reuse one instance
// for uniform OCP stages and irregular known-prefix subsegments, but it must
// supply the already selected ZOH targets in physical time order.
class PiecewiseZohPlantIntegrator {
 public:
  explicit PiecewiseZohPlantIntegrator(const ZohPlantParams& params)
      : params_(params) {
    if (!isValidZohPlantParams(params_)) {
      throw std::invalid_argument("invalid piecewise ZOH plant parameters");
    }
  }

  const ZohPlantParams& params() const noexcept { return params_; }

  PlantPropagationStatus propagateSegment(
      const PhysicalPlantState& initial, const ZohTargetSegment& segment,
      PhysicalPlantState& output) const noexcept {
    if (!isFinitePhysicalPlantState(initial)) {
      return PlantPropagationStatus::kInvalidState;
    }
    if (!std::isfinite(segment.linear_target) ||
        !std::isfinite(segment.angular_target)) {
      return PlantPropagationStatus::kInvalidTarget;
    }
    if (!std::isfinite(segment.duration_sec) ||
        segment.duration_sec < 0.0) {
      return PlantPropagationStatus::kInvalidDuration;
    }
    if (segment.duration_sec == 0.0) {
      output = initial;
      return PlantPropagationStatus::kOk;
    }

    PhysicalPlantState next = initial;
    ActualMotionState midpoint_actual;
    ActualMotionState end_actual;
    if (!piecewise_plant_detail::actualAtElapsed(
            initial.actual, segment, params_, 0.5 * segment.duration_sec,
            midpoint_actual) ||
        !piecewise_plant_detail::actualAtElapsed(
            initial.actual, segment, params_, segment.duration_sec,
            end_actual) ||
        !piecewise_plant_detail::rk4Liquid(
            initial.liquid, initial.actual, segment, params_, next.liquid)) {
      return PlantPropagationStatus::kNonFiniteOutput;
    }

    const double heading_delta =
        segment.duration_sec * midpoint_actual.angular_velocity;
    const double heading_midpoint = initial.pose.heading + 0.5 * heading_delta;
    next.pose.x = initial.pose.x +
                  segment.duration_sec * midpoint_actual.linear_velocity *
                      std::cos(heading_midpoint);
    next.pose.y = initial.pose.y +
                  segment.duration_sec * midpoint_actual.linear_velocity *
                      std::sin(heading_midpoint);
    next.pose.heading = initial.pose.heading + heading_delta;
    next.actual = end_actual;
    if (!isFinitePhysicalPlantState(next)) {
      return PlantPropagationStatus::kNonFiniteOutput;
    }
    output = next;
    return PlantPropagationStatus::kOk;
  }

  template <std::size_t SegmentCount>
  PlantPropagationStatus propagatePiecewise(
      const PhysicalPlantState& initial,
      const std::array<ZohTargetSegment, SegmentCount>& segments,
      double expected_duration_sec, double duration_tolerance_sec,
      PhysicalPlantState& output) const noexcept {
    static_assert(SegmentCount >= 1,
                  "piecewise propagation needs at least one segment slot");
    if (!isFinitePhysicalPlantState(initial)) {
      return PlantPropagationStatus::kInvalidState;
    }
    if (!std::isfinite(expected_duration_sec) ||
        expected_duration_sec <= 0.0 ||
        !std::isfinite(duration_tolerance_sec) ||
        duration_tolerance_sec < 0.0 ||
        duration_tolerance_sec >= expected_duration_sec) {
      return PlantPropagationStatus::kInvalidDuration;
    }

    bool reached_unused_slot = false;
    double total_duration = 0.0;
    for (const ZohTargetSegment& segment : segments) {
      if (!std::isfinite(segment.linear_target) ||
          !std::isfinite(segment.angular_target)) {
        return PlantPropagationStatus::kInvalidTarget;
      }
      if (!std::isfinite(segment.duration_sec) ||
          segment.duration_sec < 0.0) {
        return PlantPropagationStatus::kInvalidDuration;
      }
      if (segment.duration_sec == 0.0) {
        reached_unused_slot = true;
      } else if (reached_unused_slot) {
        return PlantPropagationStatus::kInvalidDuration;
      }
      total_duration += segment.duration_sec;
    }
    if (!std::isfinite(total_duration) ||
        std::fabs(total_duration - expected_duration_sec) >
            duration_tolerance_sec) {
      return PlantPropagationStatus::kInvalidDuration;
    }

    PhysicalPlantState current = initial;
    for (const ZohTargetSegment& segment : segments) {
      if (segment.duration_sec == 0.0) {
        continue;
      }
      PhysicalPlantState next;
      const PlantPropagationStatus status =
          propagateSegment(current, segment, next);
      if (status != PlantPropagationStatus::kOk) {
        return status;
      }
      current = next;
    }
    output = current;
    return PlantPropagationStatus::kOk;
  }

 private:
  const ZohPlantParams params_;
};

}  // namespace mainline
}  // namespace spmpc_local_planner
