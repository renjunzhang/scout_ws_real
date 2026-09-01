#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace spmpc_local_planner {
namespace mainline {

enum class RobotProgressStateIndex : std::size_t {
  kPx = 0,
  kPy,
  kTheta,
  kProgress,
  kActualLinearVelocity,
  kActualAngularVelocity,
  kCount,
};

enum class PublisherStateIndex : std::size_t {
  kPreviousLinearCommand = 0,
  kPreviousAngularCommand,
  kPreviousLinearAcceleration,
  kPreviousAngularAcceleration,
  kCount,
};

enum class LiquidStateIndex : std::size_t {
  kEtaX = 0,
  kEtaXDot,
  kEtaY,
  kEtaYDot,
  kCount,
};

enum class ControlIndex : std::size_t {
  kLinearJerk = 0,
  kAngularJerk,
  kProgressVelocity,
  kCount,
};

struct ModelContract {
  static constexpr std::size_t kRobotProgressStateCount = 6;
  static constexpr std::size_t kPublisherStateCount = 4;
  static constexpr std::size_t kLiquidStateCount = 4;
  static constexpr std::size_t kBaseStateCount = 14;
  static constexpr std::size_t kControlCount = 3;
  static constexpr std::size_t kHorizonSteps = 60;
  static constexpr std::size_t kExecutionSubsegmentSlots = 3;
  static constexpr double kNominalDtSec = 1.0 / 30.0;

  static std::size_t delayOlderCount(double maximum_delay_sec,
                                     double dt_sec = kNominalDtSec) {
    const std::size_t retained_command_count = retainedCommandCount(
        maximum_delay_sec, dt_sec);
    return retained_command_count == 0 ? 0 : retained_command_count - 1;
  }

  static std::size_t commandSelectorWidth(double maximum_delay_sec,
                                          double dt_sec = kNominalDtSec) {
    const std::size_t retained_command_count = retainedCommandCount(
        maximum_delay_sec, dt_sec);
    if (retained_command_count == std::numeric_limits<std::size_t>::max()) {
      throw std::overflow_error("delay selector width exceeds size_t");
    }
    return retained_command_count + 1;
  }

  static std::size_t stateCount(std::size_t linear_delay_older_count,
                                std::size_t angular_delay_older_count) {
    const std::size_t maximum = std::numeric_limits<std::size_t>::max();
    if (linear_delay_older_count > maximum - kBaseStateCount ||
        angular_delay_older_count >
            maximum - kBaseStateCount - linear_delay_older_count) {
      throw std::overflow_error("state dimension exceeds size_t");
    }
    return kBaseStateCount + linear_delay_older_count + angular_delay_older_count;
  }

  static std::size_t executionParameterCount(
      std::size_t linear_selector_width, std::size_t angular_selector_width) {
    const std::size_t maximum = std::numeric_limits<std::size_t>::max();
    if (angular_selector_width > maximum - linear_selector_width) {
      throw std::overflow_error("execution selector width exceeds size_t");
    }
    const std::size_t total_selector_width =
        linear_selector_width + angular_selector_width;
    if (total_selector_width >
        (maximum - 7) / kExecutionSubsegmentSlots) {
      throw std::overflow_error("execution parameter dimension exceeds size_t");
    }
    return 7 + kExecutionSubsegmentSlots * total_selector_width;
  }

 private:
  static std::size_t retainedCommandCount(double maximum_delay_sec,
                                          double dt_sec) {
    if (!std::isfinite(maximum_delay_sec) || maximum_delay_sec < 0.0 ||
        !std::isfinite(dt_sec) || dt_sec <= 0.0) {
      throw std::invalid_argument("delay bounds and dt must be finite and valid");
    }

    const double ratio = maximum_delay_sec / dt_sec;
    // Converting a floating-point value outside the destination integer range
    // is undefined.  Check the bound before applying ceil and conversion.
    const double size_max = static_cast<double>(
        std::numeric_limits<std::size_t>::max());
    // On common 64-bit platforms double(size_t::max()) rounds to 2^64.
    // Reject equality as well so that conversion can never see 2^64.
    if (!std::isfinite(ratio) || ratio >= size_max) {
      throw std::overflow_error("delay tap count exceeds size_t");
    }
    const double rounded_up = std::ceil(ratio);
    if (!std::isfinite(rounded_up) || rounded_up >= size_max) {
      throw std::overflow_error("delay tap count exceeds size_t");
    }
    return static_cast<std::size_t>(rounded_up);
  }
};

struct LiquidCostCoefficients {
  double running{0.0};
  double boundary{0.0};
};

inline LiquidCostCoefficients liquidCostCoefficients(std::size_t stage,
                                                     std::size_t liquid_intervals,
                                                     double objective_scale,
                                                     double running_total_weight,
                                                     double boundary_weight) {
  const std::size_t horizon_steps = ModelContract::kHorizonSteps;
  if (liquid_intervals == 0 || liquid_intervals >= horizon_steps) {
    throw std::invalid_argument("K_liquid must satisfy 1 <= K_liquid < N");
  }
  if (stage >= horizon_steps || !std::isfinite(objective_scale) ||
      (objective_scale != 0.0 && objective_scale != 1.0) ||
      !std::isfinite(running_total_weight) ||
      running_total_weight < 0.0 || !std::isfinite(boundary_weight) ||
      boundary_weight < 0.0) {
    throw std::invalid_argument("cost schedule inputs must be finite and in range");
  }
  LiquidCostCoefficients result;
  if (stage < liquid_intervals) {
    result.running = objective_scale * running_total_weight /
                     static_cast<double>(liquid_intervals);
  }
  if (stage == liquid_intervals) {
    result.boundary = objective_scale * boundary_weight;
  }
  if (!std::isfinite(result.running) || !std::isfinite(result.boundary)) {
    throw std::overflow_error("liquid cost coefficient is non-finite");
  }
  return result;
}

}  // namespace mainline
}  // namespace spmpc_local_planner
