#pragma once

#include "generated/slosh_kernel_contract.h"
#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace integration_policy {

inline bool validInterval(double dt) {
    return std::isfinite(dt) && dt > 1e-9 && dt <= 1.0;
}

inline int substeps(double dt) {
    return std::max(SPMPC_LIQUID_RK4_SUBSTEPS,
        static_cast<int>(std::ceil(dt / SPMPC_LIQUID_MAX_RK4_STEP_SEC - 1e-12)));
}

}  // namespace integration_policy
}  // namespace spmpc_local_planner
