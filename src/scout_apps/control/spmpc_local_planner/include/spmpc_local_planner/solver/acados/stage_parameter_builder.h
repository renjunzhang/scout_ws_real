#pragma once

#include "spmpc_local_planner/config/variant_config.h"
#include "spmpc_local_planner/phase_rejoin/types.h"

#include <array>
#include <string>
#include <vector>

namespace spmpc_local_planner {

// The generated constraint treats this value as an effectively disabled
// liquid-height cap while keeping the expression finite.
constexpr double kAcadosDisabledEtaMaxSq = 1e12;

struct AcadosSloshParameterValues {
    double two_zeta_omega_n = 0.0;
    double omega_n_sq = 0.0;
    double eta_ref = 1.0;
    double eta_dot_ref = 1.0;
    double eta_max_sq = kAcadosDisabledEtaMaxSq;
    double eta_dot_weight_ratio = 0.0;
};

// ROS- and acados-ABI-free description of every value needed to construct the
// generated solver's per-stage parameter vectors.
struct AcadosStageParameterInput {
    int horizon_steps = 0;
    bool slosh_enabled = false;
    std::array<double, 4> reference_x_coeffs{{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> reference_y_coeffs{{0.0, 0.0, 0.0, 0.0}};
    VariantConfig variant;
    double contour_error_ref = 1.0;
    double lag_error_ref = 1.0;
    double effective_v_ref = 0.0;
    AcadosSloshParameterValues slosh;
    bool have_previous_control = false;
    std::array<double, 3> previous_control{{0.0, 0.0, 0.0}};
    PhaseRejoinSolverContext phase_rejoin;
};

struct AcadosStageParameterMatrix {
    bool valid = false;
    std::string status = "NOT_BUILT";
    int stage_count = 0;
    int parameter_width = 0;
    std::vector<std::string> parameter_names;
    std::vector<double> values;

    double* stageData(int stage);
    const double* stageData(int stage) const;
    double value(int stage, int parameter_index) const;
};

// Builds the exact flattened matrix passed to generated update_params().  It
// deliberately has no dependency on generated capsule headers or acados C API,
// so B0, slosh and Phase-Rejoin layouts remain directly unit-testable.
class AcadosStageParameterBuilder {
public:
    static AcadosStageParameterMatrix build(
        const AcadosStageParameterInput& input);
};

}  // namespace spmpc_local_planner
