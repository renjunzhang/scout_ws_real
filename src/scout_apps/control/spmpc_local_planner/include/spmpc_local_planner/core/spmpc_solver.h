#pragma once

#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/core/variant_config.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/reference/reference_path.h"

namespace spmpc_local_planner {

struct SolverParams {
    double v_max = 0.8;
    double omega_max = 1.2;
    double a_max = 0.6;
    double corridor_width = 0.30;
    double lookahead_distance = 0.6;
    double goal_tolerance = 0.15;
    SloshModelParams slosh;
};

class SpmpcSolver {
public:
    virtual ~SpmpcSolver() = default;

    virtual void configure(const SolverParams& params, const VariantConfig& variant) = 0;
    virtual bool solve(const SolverInput& input, const ReferencePath& reference, SolverOutput& output) const = 0;
};

}  // namespace spmpc_local_planner
