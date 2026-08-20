#pragma once

#include "spmpc_local_planner/config/variant_config.h"
#include "spmpc_local_planner/solver/api/solver_config.h"

#include <string>

namespace spmpc_local_planner {

bool validateBackendPolicy(const SolverParams& params,
                           const VariantConfig& variant,
                           std::string& reason);

}  // namespace spmpc_local_planner
