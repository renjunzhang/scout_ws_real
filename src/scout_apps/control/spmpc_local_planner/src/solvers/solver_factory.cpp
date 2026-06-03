#include "spmpc_local_planner/solvers/solver_factory.h"

#include "spmpc_local_planner/core/rollout_sampling_solver.h"
#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"

namespace spmpc_local_planner {

bool isKnownSolverBackend(const std::string& backend) {
    return backend == kSolverBackendPrimitive ||
           backend == kSolverBackendContinuousMpccAcados;
}

std::unique_ptr<SpmpcSolver> makeSolver(const std::string& backend) {
    if (backend == kSolverBackendContinuousMpccAcados) {
        return std::unique_ptr<SpmpcSolver>(new ContinuousMpccSolverAcados());
    }
    // primitive 及未识别名称：回退到当前运动基元择优 solver。
    return std::unique_ptr<SpmpcSolver>(new RolloutSamplingSolver());
}

}  // namespace spmpc_local_planner
