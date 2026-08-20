#pragma once

#include "spmpc_local_planner/core/spmpc_solver.h"

#include <string>

namespace spmpc_local_planner {

bool validateBackendPolicy(const SolverParams& params,
                           const VariantConfig& variant,
                           std::string& reason);

}  // namespace spmpc_local_planner
