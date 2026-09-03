#include "spmpc_local_planner/solvers/mainline_solver_input_builder.h"

#include <cmath>
#include <stdexcept>

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr std::size_t offset(generated::StateOffset value) {
  return static_cast<std::size_t>(value);
}

static_assert(MainlineKnownPrefixState::kLinearOlderCount == generated::D_V,
              "linear known-prefix width differs from generated state");
static_assert(MainlineKnownPrefixState::kAngularOlderCount ==
                  generated::D_OMEGA,
              "angular known-prefix width differs from generated state");
static_assert(offset(generated::StateOffset::kStateOlderV10) -
                          offset(generated::StateOffset::kStateOlderV0) +
                      1U ==
                  generated::D_V,
              "linear older-command offsets are not contiguous");
static_assert(offset(generated::StateOffset::kStateOlderOmega22) -
                          offset(generated::StateOffset::kStateOlderOmega0) +
                      1U ==
                  generated::D_OMEGA,
              "angular older-command offsets are not contiguous");

bool finitePublisher(const AuthoritativePublisherState& publisher) {
  return std::isfinite(publisher.previous_linear_command) &&
         std::isfinite(publisher.previous_angular_command) &&
         std::isfinite(publisher.previous_linear_acceleration) &&
         std::isfinite(publisher.previous_angular_acceleration);
}

}  // namespace

MainlineState buildMainlineInitialState(
    const MainlineKnownPrefixState& known_prefix,
    double projected_progress_s) {
  if (!isFinitePhysicalPlantState(known_prefix.physical) ||
      !finitePublisher(known_prefix.publisher) ||
      !std::isfinite(projected_progress_s)) {
    throw std::invalid_argument("mainline x0 sources must be finite");
  }

  MainlineState state{};
  state[offset(generated::StateOffset::kStatePx)] =
      known_prefix.physical.pose.x;
  state[offset(generated::StateOffset::kStatePy)] =
      known_prefix.physical.pose.y;
  state[offset(generated::StateOffset::kStateTheta)] =
      known_prefix.physical.pose.heading;
  state[offset(generated::StateOffset::kStateS)] = projected_progress_s;
  state[offset(generated::StateOffset::kStateVActual)] =
      known_prefix.physical.actual.linear_velocity;
  state[offset(generated::StateOffset::kStateOmegaActual)] =
      known_prefix.physical.actual.angular_velocity;
  state[offset(generated::StateOffset::kStateQPrevV)] =
      known_prefix.publisher.previous_linear_command;
  state[offset(generated::StateOffset::kStateQPrevOmega)] =
      known_prefix.publisher.previous_angular_command;
  state[offset(generated::StateOffset::kStateAPrev)] =
      known_prefix.publisher.previous_linear_acceleration;
  state[offset(generated::StateOffset::kStateAlphaPrev)] =
      known_prefix.publisher.previous_angular_acceleration;

  const std::size_t linear_begin =
      offset(generated::StateOffset::kStateOlderV0);
  for (std::size_t index = 0; index < generated::D_V; ++index) {
    const double value = known_prefix.linear_older[index];
    if (!std::isfinite(value)) {
      throw std::invalid_argument("linear known-prefix queue must be finite");
    }
    state[linear_begin + index] = value;
  }
  const std::size_t angular_begin =
      offset(generated::StateOffset::kStateOlderOmega0);
  for (std::size_t index = 0; index < generated::D_OMEGA; ++index) {
    const double value = known_prefix.angular_older[index];
    if (!std::isfinite(value)) {
      throw std::invalid_argument("angular known-prefix queue must be finite");
    }
    state[angular_begin + index] = value;
  }

  state[offset(generated::StateOffset::kStateEtaX)] =
      known_prefix.physical.liquid.eta_x;
  state[offset(generated::StateOffset::kStateEtaXDot)] =
      known_prefix.physical.liquid.eta_x_dot;
  state[offset(generated::StateOffset::kStateEtaY)] =
      known_prefix.physical.liquid.eta_y;
  state[offset(generated::StateOffset::kStateEtaYDot)] =
      known_prefix.physical.liquid.eta_y_dot;
  return state;
}

}  // namespace mainline
}  // namespace spmpc_local_planner
