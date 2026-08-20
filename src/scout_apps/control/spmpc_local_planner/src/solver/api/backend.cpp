#include "spmpc_local_planner/solver/api/backend.h"

namespace spmpc_local_planner {

bool isKnownSolverBackend(const std::string& backend) {
    return backend == kSolverBackendPrimitive ||
           backend == kSolverBackendContinuousMpccAcados ||
           backend == kSolverBackendContinuousMpccDirectOmegaLegacy;
}

const char* solverBackendRole(const std::string& backend) {
    if (backend == kSolverBackendContinuousMpccAcados) {
        return "SPMPC mainline continuous MPCC";
    }
    if (backend == kSolverBackendContinuousMpccDirectOmegaLegacy) {
        return "RouteB diagnostic/legacy continuous MPCC, not mainline";
    }
    if (backend == kSolverBackendPrimitive) {
        return "fallback/debug rollout sampling + cost ranking, not MPCC/mainline";
    }
    return "unknown";
}

int solverBackendCode(const std::string& backend) {
    if (backend == kSolverBackendContinuousMpccAcados) {
        return 1;
    }
    if (backend == kSolverBackendContinuousMpccDirectOmegaLegacy) {
        return 2;
    }
    if (backend == kSolverBackendPrimitive) {
        return 3;
    }
    return 0;
}

}  // namespace spmpc_local_planner
