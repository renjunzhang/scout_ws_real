#pragma once

#include <cmath>
#include <stdexcept>

namespace spmpc_local_planner {
namespace mainline {

struct AuthoritativePublisherState {
  double previous_linear_command{0.0};
  double previous_angular_command{0.0};
  double previous_linear_acceleration{0.0};
  double previous_angular_acceleration{0.0};
};

struct IssueControl {
  double linear_jerk{0.0};
  double angular_jerk{0.0};
  double progress_velocity{0.0};
};

struct IssuedCommand {
  double linear_command{0.0};
  double angular_command{0.0};
  double linear_acceleration{0.0};
  double angular_acceleration{0.0};
};

// Algebraic stage output for the pre-issue transcription.  No measured robot
// velocity is accepted by this API, making measured + u0*dt unrepresentable.
inline IssuedCommand issueCommand(const AuthoritativePublisherState& state,
                                  const IssueControl& control, double dt_sec) {
  if (!std::isfinite(dt_sec) || dt_sec <= 0.0 ||
      !std::isfinite(state.previous_linear_command) ||
      !std::isfinite(state.previous_angular_command) ||
      !std::isfinite(state.previous_linear_acceleration) ||
      !std::isfinite(state.previous_angular_acceleration) ||
      !std::isfinite(control.linear_jerk) ||
      !std::isfinite(control.angular_jerk) ||
      !std::isfinite(control.progress_velocity)) {
    throw std::invalid_argument("issue map inputs must be finite and dt positive");
  }
  const double half_dt_squared = 0.5 * dt_sec * dt_sec;
  IssuedCommand result;
  result.linear_acceleration =
      state.previous_linear_acceleration + dt_sec * control.linear_jerk;
  result.angular_acceleration =
      state.previous_angular_acceleration + dt_sec * control.angular_jerk;
  result.linear_command = state.previous_linear_command +
                          dt_sec * state.previous_linear_acceleration +
                          half_dt_squared * control.linear_jerk;
  result.angular_command = state.previous_angular_command +
                           dt_sec * state.previous_angular_acceleration +
                           half_dt_squared * control.angular_jerk;
  if (!std::isfinite(result.linear_command) ||
      !std::isfinite(result.angular_command) ||
      !std::isfinite(result.linear_acceleration) ||
      !std::isfinite(result.angular_acceleration)) {
    throw std::overflow_error("issue map output is non-finite");
  }
  return result;
}

}  // namespace mainline
}  // namespace spmpc_local_planner
