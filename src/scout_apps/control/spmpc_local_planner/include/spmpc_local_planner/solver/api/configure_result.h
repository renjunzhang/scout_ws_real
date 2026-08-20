#pragma once

#include <string>

namespace spmpc_local_planner {

struct SolverConfigureResult {
    bool success = false;
    std::string status = "NOT_CONFIGURED";
    std::string detail;
};

}  // namespace spmpc_local_planner
