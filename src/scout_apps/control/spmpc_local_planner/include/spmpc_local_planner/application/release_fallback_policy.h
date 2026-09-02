#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "spmpc_local_planner/domain/release_contract.h"
#include "spmpc_local_planner/execution/command_event.h"

namespace spmpc_local_planner {
namespace mainline {

struct JerkLimitedFallbackParams {
  // Positive command, command-acceleration and command-jerk magnitudes for
  // linear (m/s) and angular (rad/s) channels.
  double maximum_linear_command{0.0};
  double maximum_angular_command{0.0};
  double maximum_linear_acceleration{0.0};
  double maximum_angular_acceleration{0.0};
  double maximum_linear_jerk{0.0};
  double maximum_angular_jerk{0.0};
};

enum class ReleaseFallbackStatus : std::uint8_t {
  kOk = 0,
  kInvalidParameters,
  kInvalidPublisherState,
  kUnrecoverablePublisherState,
  kNumericalRangeError,
};

struct ReleaseFallbackOutput {
  PlanarCommand command;
  PlanarCommandAcceleration authoritative_acceleration;
  bool stopped{false};
};

// Computes one deterministic fallback release from the last command that
// actually crossed the publisher boundary.  The policy always uses the frozen
// 30 Hz model interval; actual publication jitter remains audit evidence and
// never becomes a second integration step.
//
// Per channel, command is position and authoritative acceleration is its
// continuous-time derivative.  The policy uses the same trapezoidal
// constant-jerk map as issueCommand(), reserves enough command distance to
// slew acceleration back to zero, and never reverses command sign.  When the
// incoming acceleration points away from zero, a short command increase can
// be physically unavoidable; that acceleration is removed at the admitted
// jerk limit instead of being flipped in one cycle.
//
// A state that cannot respect command, acceleration, jerk and no-reversal
// bounds simultaneously is reported as unrecoverable.  The release owner must
// classify that result as a hard stop rather than claim a smooth fallback.
// Any non-kOk result leaves output unchanged.
class JerkLimitedFallbackPolicy {
 public:
  explicit JerkLimitedFallbackPolicy(
      const JerkLimitedFallbackParams& parameters) noexcept
      : parameters_(parameters) {}

  const JerkLimitedFallbackParams& parameters() const noexcept {
    return parameters_;
  }

  static constexpr double timeStepSeconds() noexcept {
    return static_cast<double>(ReleaseGridContract::kPeriodNumeratorSeconds) /
           static_cast<double>(ReleaseGridContract::kPeriodDenominator);
  }

  ReleaseFallbackStatus computeNext(
      const AuthoritativePublisherState& publisher_state,
      ReleaseFallbackOutput& output) const noexcept {
    if (!validParameters(parameters_)) {
      return ReleaseFallbackStatus::kInvalidParameters;
    }
    if (!finitePublisherState(publisher_state) ||
        std::fabs(publisher_state.previous_linear_command) >
            parameters_.maximum_linear_command ||
        std::fabs(publisher_state.previous_angular_command) >
            parameters_.maximum_angular_command ||
        std::fabs(publisher_state.previous_linear_acceleration) >
            parameters_.maximum_linear_acceleration ||
        std::fabs(publisher_state.previous_angular_acceleration) >
            parameters_.maximum_angular_acceleration) {
      return ReleaseFallbackStatus::kInvalidPublisherState;
    }

    ReleaseFallbackOutput candidate;
    const ReleaseFallbackStatus linear_status = computeChannel(
        publisher_state.previous_linear_command,
        publisher_state.previous_linear_acceleration,
        parameters_.maximum_linear_command,
        parameters_.maximum_linear_acceleration,
        parameters_.maximum_linear_jerk, candidate.command.linear,
        candidate.authoritative_acceleration.linear);
    if (linear_status != ReleaseFallbackStatus::kOk) {
      return linear_status;
    }
    const ReleaseFallbackStatus angular_status = computeChannel(
        publisher_state.previous_angular_command,
        publisher_state.previous_angular_acceleration,
        parameters_.maximum_angular_command,
        parameters_.maximum_angular_acceleration,
        parameters_.maximum_angular_jerk, candidate.command.angular,
        candidate.authoritative_acceleration.angular);
    if (angular_status != ReleaseFallbackStatus::kOk) {
      return angular_status;
    }

    candidate.stopped =
        candidate.command.linear == 0.0 &&
        candidate.command.angular == 0.0 &&
        candidate.authoritative_acceleration.linear == 0.0 &&
        candidate.authoritative_acceleration.angular == 0.0;
    output = candidate;
    return ReleaseFallbackStatus::kOk;
  }

 private:
  static bool validParameters(
      const JerkLimitedFallbackParams& parameters) noexcept {
    return std::isfinite(parameters.maximum_linear_command) &&
           std::isfinite(parameters.maximum_angular_command) &&
           std::isfinite(parameters.maximum_linear_acceleration) &&
           std::isfinite(parameters.maximum_angular_acceleration) &&
           std::isfinite(parameters.maximum_linear_jerk) &&
           std::isfinite(parameters.maximum_angular_jerk) &&
           parameters.maximum_linear_command > 0.0 &&
           parameters.maximum_angular_command > 0.0 &&
           parameters.maximum_linear_acceleration > 0.0 &&
           parameters.maximum_angular_acceleration > 0.0 &&
           parameters.maximum_linear_jerk > 0.0 &&
           parameters.maximum_angular_jerk > 0.0;
  }

  static bool finitePublisherState(
      const AuthoritativePublisherState& state) noexcept {
    return std::isfinite(state.previous_linear_command) &&
           std::isfinite(state.previous_angular_command) &&
           std::isfinite(state.previous_linear_acceleration) &&
           std::isfinite(state.previous_angular_acceleration);
  }

  // Exact distance consumed after a negative endpoint acceleration while it
  // is slewed to zero on subsequent fixed-rate releases.  A fractional final
  // jerk step is spread across its full 30 Hz interval, matching the
  // trapezoidal constant-jerk issue map.
  static bool futureStoppingDistance(double normalized_acceleration,
                                     double acceleration_step,
                                     double& distance) noexcept {
    if (normalized_acceleration >= 0.0) {
      distance = 0.0;
      return true;
    }

    const double ratio = -normalized_acceleration / acceleration_step;
    if (!std::isfinite(ratio)) {
      return false;
    }
    const double whole_steps = std::floor(ratio);
    const double fractional_step = ratio - whole_steps;
    const double staircase_factor =
        ratio * ratio + fractional_step * (1.0 - fractional_step);
    distance = 0.5 * timeStepSeconds() * acceleration_step *
               staircase_factor;
    return std::isfinite(whole_steps) && std::isfinite(staircase_factor) &&
           std::isfinite(distance) && distance >= 0.0;
  }

  static bool viabilityMargin(double remaining_command,
                              double normalized_previous_acceleration,
                              double normalized_next_acceleration,
                              double acceleration_step,
                              double& margin) noexcept {
    double future_distance = 0.0;
    if (!futureStoppingDistance(normalized_next_acceleration,
                                acceleration_step, future_distance)) {
      return false;
    }
    const double next_command =
        remaining_command +
        0.5 * timeStepSeconds() *
            (normalized_previous_acceleration +
             normalized_next_acceleration);
    margin = next_command - future_distance;
    return std::isfinite(next_command) && std::isfinite(margin);
  }

  static ReleaseFallbackStatus computeChannel(
      double previous_command, double previous_acceleration,
      double maximum_command, double maximum_acceleration,
      double maximum_jerk, double& next_command,
      double& next_acceleration) noexcept {
    if (previous_command == 0.0) {
      if (previous_acceleration != 0.0) {
        return ReleaseFallbackStatus::kUnrecoverablePublisherState;
      }
      next_command = 0.0;
      next_acceleration = 0.0;
      return ReleaseFallbackStatus::kOk;
    }

    const double dt_sec = timeStepSeconds();
    const double acceleration_step = maximum_jerk * dt_sec;
    if (!std::isfinite(acceleration_step) || acceleration_step <= 0.0) {
      return ReleaseFallbackStatus::kNumericalRangeError;
    }

    const double direction = previous_command > 0.0 ? 1.0 : -1.0;
    const double remaining_command = std::fabs(previous_command);
    const double normalized_previous_acceleration =
        direction * previous_acceleration;
    const double minimum_admissible_acceleration =
        std::max(-maximum_acceleration,
                 normalized_previous_acceleration - acceleration_step);
    double maximum_admissible_acceleration =
        std::min(maximum_acceleration,
                 normalized_previous_acceleration + acceleration_step);
    if (normalized_previous_acceleration <= 0.0) {
      maximum_admissible_acceleration =
          std::min(maximum_admissible_acceleration, 0.0);
    }

    const double command_headroom = maximum_command - remaining_command;
    const double command_bound_acceleration =
        (2.0 * command_headroom / dt_sec) -
        normalized_previous_acceleration;
    maximum_admissible_acceleration =
        std::min(maximum_admissible_acceleration,
                 command_bound_acceleration);
    if (!std::isfinite(minimum_admissible_acceleration) ||
        !std::isfinite(maximum_admissible_acceleration) ||
        minimum_admissible_acceleration >
            maximum_admissible_acceleration) {
      return ReleaseFallbackStatus::kUnrecoverablePublisherState;
    }

    double maximum_margin = 0.0;
    if (!viabilityMargin(remaining_command,
                         normalized_previous_acceleration,
                         maximum_admissible_acceleration,
                         acceleration_step, maximum_margin)) {
      return ReleaseFallbackStatus::kNumericalRangeError;
    }
    if (maximum_margin < 0.0) {
      return ReleaseFallbackStatus::kUnrecoverablePublisherState;
    }

    double selected_acceleration = minimum_admissible_acceleration;
    double minimum_margin = 0.0;
    if (!viabilityMargin(remaining_command,
                         normalized_previous_acceleration,
                         minimum_admissible_acceleration,
                         acceleration_step, minimum_margin)) {
      return ReleaseFallbackStatus::kNumericalRangeError;
    }
    if (minimum_margin < 0.0) {
      double unsafe_acceleration = minimum_admissible_acceleration;
      double safe_acceleration = maximum_admissible_acceleration;
      // Fixed work, no allocation: locate the strongest endpoint acceleration
      // whose complete jerk-limited stop fits in the remaining command.
      for (std::size_t iteration = 0; iteration < 64; ++iteration) {
        const double midpoint =
            0.5 * (unsafe_acceleration + safe_acceleration);
        double midpoint_margin = 0.0;
        if (!viabilityMargin(remaining_command,
                             normalized_previous_acceleration, midpoint,
                             acceleration_step, midpoint_margin)) {
          return ReleaseFallbackStatus::kNumericalRangeError;
        }
        if (midpoint_margin >= 0.0) {
          safe_acceleration = midpoint;
        } else {
          unsafe_acceleration = midpoint;
        }
      }
      selected_acceleration = safe_acceleration;
    }

    // Keep published double state on the interior side of the recoverable
    // boundary.  Moving a few representable values toward the already-proved
    // safe upper endpoint only weakens braking; it cannot violate jerk,
    // acceleration, command or no-reversal bounds.  This is deliberately not
    // a symmetric epsilon that could admit a negative stopping margin.
    for (std::size_t step = 0;
         step < 8 &&
         selected_acceleration < maximum_admissible_acceleration;
         ++step) {
      selected_acceleration =
          std::nextafter(selected_acceleration,
                         maximum_admissible_acceleration);
    }

    double selected_margin = 0.0;
    if (!viabilityMargin(remaining_command,
                         normalized_previous_acceleration,
                         selected_acceleration, acceleration_step,
                         selected_margin)) {
      return ReleaseFallbackStatus::kNumericalRangeError;
    }
    if (selected_margin < 0.0) {
      return ReleaseFallbackStatus::kUnrecoverablePublisherState;
    }

    double next_magnitude =
        remaining_command +
        0.5 * dt_sec *
            (normalized_previous_acceleration + selected_acceleration);
    if (!std::isfinite(next_magnitude)) {
      return ReleaseFallbackStatus::kNumericalRangeError;
    }
    if (next_magnitude < 0.0 ||
        next_magnitude > maximum_command) {
      return ReleaseFallbackStatus::kNumericalRangeError;
    }

    // A conservative safety bias can leave a positive sub-ulp-scale terminal
    // residue.  Canonicalize only an already viable interior state, and only
    // when both the command-map residual and endpoint acceleration are within
    // the explicitly bounded floating-point representation tolerance.  This
    // never turns a negative stopping margin into success.
    const double command_arithmetic_scale =
        std::max(std::min(maximum_command, 1.0),
                 std::max(remaining_command,
                          dt_sec *
                              std::max(
                                  std::fabs(
                                      normalized_previous_acceleration),
                                  std::fabs(selected_acceleration))));
    const double acceleration_arithmetic_scale =
        std::max(std::min(maximum_acceleration, 1.0),
                 std::max(acceleration_step,
                          std::max(
                              std::fabs(normalized_previous_acceleration),
                              std::fabs(selected_acceleration))));
    const double canonical_command_tolerance =
        32.0 * std::numeric_limits<double>::epsilon() *
        command_arithmetic_scale;
    const double canonical_acceleration_tolerance =
        32.0 * std::numeric_limits<double>::epsilon() *
        acceleration_arithmetic_scale;
    if (next_magnitude <= canonical_command_tolerance &&
        std::fabs(selected_acceleration) <=
            canonical_acceleration_tolerance &&
        std::fabs(normalized_previous_acceleration) <= acceleration_step) {
      next_magnitude = 0.0;
      selected_acceleration = 0.0;
    }

    next_command = direction * next_magnitude;
    next_acceleration = direction * selected_acceleration;
    if (!std::isfinite(next_command) || !std::isfinite(next_acceleration)) {
      return ReleaseFallbackStatus::kNumericalRangeError;
    }
    return ReleaseFallbackStatus::kOk;
  }

  JerkLimitedFallbackParams parameters_;
};

static_assert(std::is_trivially_copyable<JerkLimitedFallbackParams>::value,
              "fallback parameters must have bounded trivial copies");
static_assert(std::is_trivially_copyable<ReleaseFallbackOutput>::value,
              "fallback output must have bounded trivial copies");
static_assert(std::is_trivially_copyable<JerkLimitedFallbackPolicy>::value,
              "fallback policy must not own dynamic state");

}  // namespace mainline
}  // namespace spmpc_local_planner
