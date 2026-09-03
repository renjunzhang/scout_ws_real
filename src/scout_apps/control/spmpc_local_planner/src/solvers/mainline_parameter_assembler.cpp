#include "spmpc_local_planner/solvers/mainline_parameter_assembler.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "spmpc_local_planner/domain/release_contract.h"

namespace spmpc_local_planner {
namespace mainline {
namespace {

constexpr std::size_t offset(generated::ParameterOffset value) {
  return static_cast<std::size_t>(value);
}

constexpr double kReleasePeriodSec =
    static_cast<double>(ReleaseGridContract::kPeriodNumeratorSeconds) /
    static_cast<double>(ReleaseGridContract::kPeriodDenominator);

static_assert(generated::PARAMETER_VECTOR_COUNT == generated::N + 1U,
              "parameter horizon must include the terminal row");
static_assert(offset(generated::ParameterOffset::kParameterActSelV00) +
                      generated::EXECUTION_SUBSEGMENT_SLOTS * generated::NQ_V ==
                  offset(generated::ParameterOffset::kParameterActSelOmega00),
              "linear selector block is not contiguous");
static_assert(offset(generated::ParameterOffset::kParameterActSelOmega00) +
                      generated::EXECUTION_SUBSEGMENT_SLOTS *
                          generated::NQ_OMEGA ==
                  generated::kParameterBlockExecutionPrefixEnd,
              "angular selector block is not contiguous");

bool finite(double value) { return std::isfinite(value); }
bool positive(double value) { return finite(value) && value > 0.0; }
bool nonnegative(double value) { return finite(value) && value >= 0.0; }

template <typename Values>
bool allFinite(const Values& values) {
  return std::all_of(values.begin(), values.end(), finite);
}

template <typename Values>
bool allNonnegative(const Values& values) {
  return std::all_of(values.begin(), values.end(), nonnegative);
}

void validate(const MainlineRuntimeParameterValues& values) {
  if (!positive(values.dt_sec) || !nonnegative(values.duration_tolerance_sec) ||
      values.duration_tolerance_sec >= values.dt_sec ||
      std::fabs(values.dt_sec - kReleasePeriodSec) >
          values.duration_tolerance_sec ||
      !isValidFopdtChannel(values.linear_actuator) ||
      !isValidFopdtChannel(values.angular_actuator) ||
      !values.delay_schedule.valid(values.dt_sec,
                                   values.duration_tolerance_sec)) {
    throw std::invalid_argument("mainline execution parameters are invalid");
  }
  const double inverse_linear_tau = 1.0 / values.linear_actuator.tau_sec;
  const double inverse_angular_tau = 1.0 / values.angular_actuator.tau_sec;
  if (!positive(inverse_linear_tau) || !positive(inverse_angular_tau) ||
      !finite(values.reference.s_origin) ||
      !positive(values.reference.s_scale) ||
      !allFinite(values.reference.x_coefficients) ||
      !allFinite(values.reference.y_coefficients) ||
      !allNonnegative(values.reference.speed)) {
    throw std::invalid_argument("mainline actuator/reference parameters are invalid");
  }
  const double slosh_damping = 2.0 * values.slosh.damping_ratio *
                               values.slosh.omega_n_rad_per_sec;
  const double slosh_stiffness = values.slosh.omega_n_rad_per_sec *
                                 values.slosh.omega_n_rad_per_sec;
  if (!positive(values.slosh.omega_n_rad_per_sec) ||
      !nonnegative(values.slosh.damping_ratio) ||
      !positive(values.slosh.kappa_x) || !positive(values.slosh.kappa_y) ||
      !positive(values.slosh.eta_ref) ||
      !nonnegative(values.slosh.running_eta_dot_ratio) ||
      !nonnegative(slosh_damping) || !positive(slosh_stiffness)) {
    throw std::invalid_argument("mainline slosh parameters are invalid");
  }
  const std::array<double, 9> normalizations{{
      values.normalization.contour, values.normalization.lag,
      values.normalization.v_actual, values.normalization.omega_actual,
      values.normalization.v_s, values.normalization.a_issue,
      values.normalization.alpha_issue, values.normalization.jerk_v,
      values.normalization.jerk_omega}};
  if (!std::all_of(normalizations.begin(), normalizations.end(), positive)) {
    throw std::invalid_argument("mainline normalizations must be positive");
  }
  const std::array<double, 13> weights{{
      values.running_weights.contour, values.running_weights.lag,
      values.running_weights.progress, values.running_weights.v_actual,
      values.running_weights.v_s, values.running_weights.a_issue,
      values.running_weights.alpha_issue, values.running_weights.jerk_v,
      values.running_weights.jerk_omega, values.terminal_weights.contour,
      values.terminal_weights.lag, values.terminal_weights.v_actual,
      values.terminal_weights.omega_actual}};
  if (!allNonnegative(weights) ||
      values.liquid_cost.trusted_intervals == 0U ||
      values.liquid_cost.trusted_intervals >= generated::N ||
      !nonnegative(values.liquid_cost.running_total_weight) ||
      !nonnegative(values.liquid_cost.boundary_weight)) {
    throw std::invalid_argument("mainline weight/cost parameters are invalid");
  }
  (void)liquidObjectiveScale(values.liquid_cost.condition);
}

void assign(MainlineStageParameters& row,
            std::array<bool, generated::NP>& assigned,
            generated::ParameterOffset field, double value) {
  const std::size_t index = offset(field);
  if (assigned[index] || !finite(value)) {
    throw std::logic_error("mainline parameter assignment is duplicate/non-finite");
  }
  row[index] = value;
  assigned[index] = true;
}

void assignIndex(MainlineStageParameters& row,
                 std::array<bool, generated::NP>& assigned,
                 std::size_t index, double value) {
  if (index >= row.size() || assigned[index] || !finite(value)) {
    throw std::logic_error("mainline indexed parameter assignment is invalid");
  }
  row[index] = value;
  assigned[index] = true;
}

}  // namespace

MainlineParameterHorizon assembleMainlineParameters(
    const MainlineRuntimeParameterValues& values) {
  validate(values);
  MainlineParameterHorizon horizon{};
  const double inverse_linear_tau = 1.0 / values.linear_actuator.tau_sec;
  const double inverse_angular_tau = 1.0 / values.angular_actuator.tau_sec;
  const double slosh_damping = 2.0 * values.slosh.damping_ratio *
                               values.slosh.omega_n_rad_per_sec;
  const double slosh_stiffness = values.slosh.omega_n_rad_per_sec *
                                 values.slosh.omega_n_rad_per_sec;
  const double liquid_scale = liquidObjectiveScale(values.liquid_cost.condition);
  const double liquid_running =
      liquid_scale * values.liquid_cost.running_total_weight /
      static_cast<double>(values.liquid_cost.trusted_intervals);
  const double liquid_boundary =
      liquid_scale * values.liquid_cost.boundary_weight;
  if (!finite(liquid_running) || !finite(liquid_boundary)) {
    throw std::overflow_error("mainline liquid cost coefficients are non-finite");
  }

  for (std::size_t stage = 0; stage <= generated::N; ++stage) {
    MainlineStageParameters& row = horizon[stage];
    std::array<bool, generated::NP> assigned{};
    assign(row, assigned, generated::ParameterOffset::kParameterActInvTauV,
           inverse_linear_tau);
    assign(row, assigned, generated::ParameterOffset::kParameterActGainV,
           values.linear_actuator.gain);
    assign(row, assigned,
           generated::ParameterOffset::kParameterActInvTauOmega,
           inverse_angular_tau);
    assign(row, assigned, generated::ParameterOffset::kParameterActGainOmega,
           values.angular_actuator.gain);

    const std::size_t segment_begin =
        offset(generated::ParameterOffset::kParameterActSegDt0);
    const std::size_t linear_selector_begin =
        offset(generated::ParameterOffset::kParameterActSelV00);
    const std::size_t angular_selector_begin =
        offset(generated::ParameterOffset::kParameterActSelOmega00);
    for (std::size_t slot = 0; slot < generated::EXECUTION_SUBSEGMENT_SLOTS;
         ++slot) {
      assignIndex(row, assigned, segment_begin + slot,
                  values.delay_schedule.duration[slot]);
      for (std::size_t selector = 0; selector < generated::NQ_V; ++selector) {
        assignIndex(row, assigned,
                    linear_selector_begin + slot * generated::NQ_V + selector,
                    values.delay_schedule.linear_selector[slot][selector]);
      }
      for (std::size_t selector = 0; selector < generated::NQ_OMEGA;
           ++selector) {
        assignIndex(row, assigned,
                    angular_selector_begin + slot * generated::NQ_OMEGA +
                        selector,
                    values.delay_schedule.angular_selector[slot][selector]);
      }
    }

    assign(row, assigned, generated::ParameterOffset::kParameterRefSOrigin,
           values.reference.s_origin);
    assign(row, assigned, generated::ParameterOffset::kParameterRefSScale,
           values.reference.s_scale);
    const std::size_t reference_x_begin =
        offset(generated::ParameterOffset::kParameterRefXCoeff0);
    const std::size_t reference_y_begin =
        offset(generated::ParameterOffset::kParameterRefYCoeff0);
    for (std::size_t index = 0; index < 4U; ++index) {
      assignIndex(row, assigned, reference_x_begin + index,
                  values.reference.x_coefficients[index]);
      assignIndex(row, assigned, reference_y_begin + index,
                  values.reference.y_coefficients[index]);
    }
    assign(row, assigned, generated::ParameterOffset::kParameterRefSpeed,
           values.reference.speed[stage]);

    assign(row, assigned,
           generated::ParameterOffset::kParameterSloshTwoZetaOmegaN,
           slosh_damping);
    assign(row, assigned, generated::ParameterOffset::kParameterSloshOmegaNSq,
           slosh_stiffness);
    assign(row, assigned, generated::ParameterOffset::kParameterSloshKappaX,
           values.slosh.kappa_x);
    assign(row, assigned, generated::ParameterOffset::kParameterSloshKappaY,
           values.slosh.kappa_y);
    assign(row, assigned, generated::ParameterOffset::kParameterSloshEtaRef,
           values.slosh.eta_ref);
    assign(row, assigned,
           generated::ParameterOffset::kParameterSloshRunningEtaDotRatio,
           values.slosh.running_eta_dot_ratio);

    const auto set = [&](generated::ParameterOffset field, double value) {
      assign(row, assigned, field, value);
    };
    set(generated::ParameterOffset::kParameterNormContour,
        values.normalization.contour);
    set(generated::ParameterOffset::kParameterNormLag,
        values.normalization.lag);
    set(generated::ParameterOffset::kParameterNormVActual,
        values.normalization.v_actual);
    set(generated::ParameterOffset::kParameterNormOmegaActual,
        values.normalization.omega_actual);
    set(generated::ParameterOffset::kParameterNormVS,
        values.normalization.v_s);
    set(generated::ParameterOffset::kParameterNormAIssue,
        values.normalization.a_issue);
    set(generated::ParameterOffset::kParameterNormAlphaIssue,
        values.normalization.alpha_issue);
    set(generated::ParameterOffset::kParameterNormJerkV,
        values.normalization.jerk_v);
    set(generated::ParameterOffset::kParameterNormJerkOmega,
        values.normalization.jerk_omega);
    set(generated::ParameterOffset::kParameterWeightContour,
        values.running_weights.contour);
    set(generated::ParameterOffset::kParameterWeightLag,
        values.running_weights.lag);
    set(generated::ParameterOffset::kParameterWeightProgress,
        values.running_weights.progress);
    set(generated::ParameterOffset::kParameterWeightVActual,
        values.running_weights.v_actual);
    set(generated::ParameterOffset::kParameterWeightVS,
        values.running_weights.v_s);
    set(generated::ParameterOffset::kParameterWeightAIssue,
        values.running_weights.a_issue);
    set(generated::ParameterOffset::kParameterWeightAlphaIssue,
        values.running_weights.alpha_issue);
    set(generated::ParameterOffset::kParameterWeightJerkV,
        values.running_weights.jerk_v);
    set(generated::ParameterOffset::kParameterWeightJerkOmega,
        values.running_weights.jerk_omega);
    set(generated::ParameterOffset::kParameterWeightTerminalContour,
        values.terminal_weights.contour);
    set(generated::ParameterOffset::kParameterWeightTerminalLag,
        values.terminal_weights.lag);
    set(generated::ParameterOffset::kParameterWeightTerminalVActual,
        values.terminal_weights.v_actual);
    set(generated::ParameterOffset::kParameterWeightTerminalOmegaActual,
        values.terminal_weights.omega_actual);
    set(generated::ParameterOffset::kParameterLiquidRunCoeff,
        stage < values.liquid_cost.trusted_intervals ? liquid_running : 0.0);
    set(generated::ParameterOffset::kParameterLiquidBoundaryCoeff,
        stage == values.liquid_cost.trusted_intervals ? liquid_boundary
                                                      : 0.0);

    if (!std::all_of(assigned.begin(), assigned.end(), [](bool value) {
          return value;
        })) {
      throw std::logic_error("mainline parameter row is not fully assigned");
    }
  }
  return horizon;
}

}  // namespace mainline
}  // namespace spmpc_local_planner
