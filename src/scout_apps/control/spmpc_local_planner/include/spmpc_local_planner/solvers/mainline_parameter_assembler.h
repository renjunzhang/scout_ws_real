#pragma once

#include <array>
#include <cstddef>

#include "spmpc_local_planner/domain/mainline_types.h"
#include "spmpc_local_planner/execution/actuator_response_params.h"
#include "spmpc_local_planner/execution/discrete_delay_queue.h"
#include "spmpc_local_planner/solvers/mainline_mpcc_solver_acados.h"

namespace spmpc_local_planner {
namespace mainline {

using MainlineDelaySchedule =
    CombinedDelaySchedule<generated::NQ_V, generated::NQ_OMEGA>;

struct MainlineReferenceParameters {
  double s_origin{0.0};
  double s_scale{0.0};
  std::array<double, 4> x_coefficients{};
  std::array<double, 4> y_coefficients{};
  std::array<double, generated::PARAMETER_VECTOR_COUNT> speed{};
};

struct MainlineSloshParameters {
  double omega_n_rad_per_sec{0.0};
  double damping_ratio{0.0};
  double kappa_x{0.0};
  double kappa_y{0.0};
  double eta_ref{0.0};
  double running_eta_dot_ratio{0.0};
};

struct MainlineNormalizationParameters {
  double contour{0.0};
  double lag{0.0};
  double v_actual{0.0};
  double omega_actual{0.0};
  double v_s{0.0};
  double a_issue{0.0};
  double alpha_issue{0.0};
  double jerk_v{0.0};
  double jerk_omega{0.0};
};

struct MainlineRunningWeights {
  double contour{0.0};
  double lag{0.0};
  double progress{0.0};
  double v_actual{0.0};
  double v_s{0.0};
  double a_issue{0.0};
  double alpha_issue{0.0};
  double jerk_v{0.0};
  double jerk_omega{0.0};
};

struct MainlineTerminalWeights {
  double contour{0.0};
  double lag{0.0};
  double v_actual{0.0};
  double omega_actual{0.0};
};

struct MainlineLiquidCostParameters {
  ExperimentCondition condition{ExperimentCondition::kB0};
  std::size_t trusted_intervals{0};
  double running_total_weight{0.0};
  double boundary_weight{0.0};
};

struct MainlineRuntimeParameterValues {
  double dt_sec{0.0};
  double duration_tolerance_sec{0.0};
  FopdtChannelParams linear_actuator;
  FopdtChannelParams angular_actuator;
  MainlineDelaySchedule delay_schedule;
  MainlineReferenceParameters reference;
  MainlineSloshParameters slosh;
  MainlineNormalizationParameters normalization;
  MainlineRunningWeights running_weights;
  MainlineTerminalWeights terminal_weights;
  MainlineLiquidCostParameters liquid_cost;
};

// Assigns every field of p[0]..p[N] from one explicit immutable trial input.
MainlineParameterHorizon assembleMainlineParameters(
    const MainlineRuntimeParameterValues& values);

}  // namespace mainline
}  // namespace spmpc_local_planner
