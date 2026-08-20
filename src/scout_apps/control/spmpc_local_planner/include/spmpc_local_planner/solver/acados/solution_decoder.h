#pragma once

#include "spmpc_local_planner/solver/api/solver_io.h"
#include "spmpc_local_planner/core/variant_config.h"
#include "spmpc_local_planner/reference/reference_path.h"
#include "spmpc_local_planner/solver/api/solver_config.h"
#include "spmpc_local_planner/warm_start/warm_start_output.h"

#include <array>
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct AcadosRawSolution {
    int horizon_steps = 0;
    int state_width = 10;
    int control_width = 3;
    std::vector<double> states;
    std::vector<double> controls;

    const double* stateData(int stage) const;
    const double* controlData(int stage) const;
};

struct AcadosSolutionDecoderInput {
    const AcadosRawSolution* raw_solution = nullptr;
    const SolverInput* solver_input = nullptr;
    const ReferencePath* reference = nullptr;
    const SolverParams* params = nullptr;
    const VariantConfig* variant = nullptr;
    std::array<double, 4> reference_x_coeffs{{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 4> reference_y_coeffs{{0.0, 0.0, 0.0, 0.0}};
    double contour_error_ref = 1.0;
    double lag_error_ref = 1.0;
    double effective_v_ref = 0.0;
    double height_coeff = 1.0;
    double eta_ref = 1.0;
    double eta_dot_ref = 1.0;
    bool slosh_enabled = false;
    bool have_previous_control = false;
    std::array<double, 3> previous_control{{0.0, 0.0, 0.0}};
};

struct AcadosSolutionDecodeResult {
    bool valid = false;
    std::string status = "NOT_DECODED";
    std::array<double, 3> first_control{{0.0, 0.0, 0.0}};
    WarmStartOutput solved_warm_start;
};

// Converts raw generated x/u matrices into the stable planner-domain output.
// No acados headers or C ABI symbols are visible at this boundary.
class AcadosSolutionDecoder {
public:
    static AcadosSolutionDecodeResult decode(
        const AcadosSolutionDecoderInput& input,
        SolverOutput& output);
};

}  // namespace spmpc_local_planner
