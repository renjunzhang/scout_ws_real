#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace spmpc_local_planner {
namespace mainline {

// FOPDT parameters for one independent channel.  Dead time is handled by the
// fixed-width DiscreteDelayQueue; this type contains only the first-order
// response parameters used after a target becomes effective.
struct FopdtChannelParams {
  double tau_sec{0.0};
  double gain{0.0};
};

// Startup/configuration representation.  Keeping delay with the channel
// parameters makes the six externally visible values easy to audit, while
// the numerical kernel below consumes the delay-free pair explicitly.
struct ActuatorResponseParams {
  double delay_sec{0.0};
  double tau_sec{0.0};
  double gain{0.0};
};

enum class FopdtStepStatus : std::uint8_t {
  kOk = 0,
  kInvalidParams,
  kInvalidState,
  kNonFiniteOutput,
};

inline bool isValidFopdtChannel(const FopdtChannelParams& params) noexcept {
  return std::isfinite(params.tau_sec) && params.tau_sec > 0.0 &&
         std::isfinite(params.gain) && params.gain > 0.0;
}

inline bool isValidActuatorResponseParams(
    const ActuatorResponseParams& params) noexcept {
  return std::isfinite(params.delay_sec) && params.delay_sec >= 0.0 &&
         isValidFopdtChannel(FopdtChannelParams{params.tau_sec, params.gain});
}

inline void validateActuatorResponseParams(
    const ActuatorResponseParams& params) {
  if (!isValidActuatorResponseParams(params)) {
    throw std::invalid_argument(
        "actuator delay must be nonnegative, tau positive, and gain positive");
  }
}

inline void validateFopdtChannel(const FopdtChannelParams& params) {
  if (!isValidFopdtChannel(params)) {
    throw std::invalid_argument("FOPDT tau and gain must be finite and positive");
  }
}

// Exact response over a constant-target interval:
//   x_next = exp(-delta/tau) x + (1-exp(-delta/tau)) gain target.
// This status-returning kernel is suitable for fail-closed rollout paths and
// leaves actual_next untouched on failure.
inline FopdtStepStatus fopdtStep(
    double actual_current, double q_target, double delta_sec,
    const FopdtChannelParams& params, double& actual_next) noexcept {
  if (!isValidFopdtChannel(params)) {
    return FopdtStepStatus::kInvalidParams;
  }
  if (!std::isfinite(actual_current) || !std::isfinite(q_target) ||
      !std::isfinite(delta_sec) || delta_sec < 0.0) {
    return FopdtStepStatus::kInvalidState;
  }
  if (delta_sec == 0.0) {
    actual_next = actual_current;
    return FopdtStepStatus::kOk;
  }

  const double exponent = delta_sec / params.tau_sec;
  const double steady_state = params.gain * q_target;
  if (!std::isfinite(exponent) || !std::isfinite(steady_state)) {
    return FopdtStepStatus::kNonFiniteOutput;
  }
  const double rho = std::exp(-exponent);
  if (!std::isfinite(rho)) {
    return FopdtStepStatus::kNonFiniteOutput;
  }
  // -expm1(-x) retains precision when a schedule contains a very short
  // subsegment, where 1-exp(-x) would lose the effective gain.
  const double one_minus_rho = -std::expm1(-exponent);
  if (!std::isfinite(one_minus_rho)) {
    return FopdtStepStatus::kNonFiniteOutput;
  }
  const double next = rho * actual_current + one_minus_rho * steady_state;
  if (!std::isfinite(next)) {
    return FopdtStepStatus::kNonFiniteOutput;
  }
  actual_next = next;
  return FopdtStepStatus::kOk;
}

// Throwing convenience wrapper for configuration/replay code that treats a
// failed numerical step as a hard contract violation.
inline double stepFopdt(double actual, double target, double duration_sec,
                        const ActuatorResponseParams& params) {
  validateActuatorResponseParams(params);
  double next = actual;
  const FopdtStepStatus status = fopdtStep(
      actual, target, duration_sec,
      FopdtChannelParams{params.tau_sec, params.gain}, next);
  if (status == FopdtStepStatus::kInvalidParams ||
      status == FopdtStepStatus::kInvalidState) {
    throw std::invalid_argument("invalid FOPDT step input");
  }
  if (status != FopdtStepStatus::kOk) {
    throw std::overflow_error("FOPDT step produced a non-finite state");
  }
  return next;
}

inline double fopdtAcceleration(double actual, double target,
                                const ActuatorResponseParams& params) {
  validateActuatorResponseParams(params);
  if (!std::isfinite(actual) || !std::isfinite(target)) {
    throw std::invalid_argument("FOPDT state and target must be finite");
  }
  const double steady_state = params.gain * target;
  const double numerator = steady_state - actual;
  const double acceleration = numerator / params.tau_sec;
  if (!std::isfinite(steady_state) || !std::isfinite(numerator) ||
      !std::isfinite(acceleration)) {
    throw std::overflow_error("FOPDT acceleration produced a non-finite state");
  }
  return acceleration;
}

}  // namespace mainline
}  // namespace spmpc_local_planner
