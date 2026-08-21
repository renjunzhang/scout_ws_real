#include "spmpc_local_planner/solvers/solver_factory.h"

#include "spmpc_local_planner/solvers/rollout_sampling_solver.h"
#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"
#include "spmpc_local_planner/solvers/delay_augmented_phase_online_solver.h"
#ifdef SPMPC_BUILD_LEGACY_BACKEND
#include "spmpc_local_planner/solvers/continuous_mpcc_direct_omega_legacy_solver_acados.h"
#endif

#include <stdexcept>

namespace spmpc_local_planner {

std::unique_ptr<SpmpcSolver> makeSolver(const std::string& backend) {
    if (backend == kSolverBackendContinuousMpccAcados) {
        return std::unique_ptr<SpmpcSolver>(new ContinuousMpccSolverAcados());
    }
    if (backend == kSolverBackendDelayAugmentedPhaseAcados) {
        return std::unique_ptr<SpmpcSolver>(
            new DelayAugmentedPhaseOnlineSolver());
    }
    if (backend == kSolverBackendContinuousMpccDirectOmegaLegacy) {
#ifdef SPMPC_BUILD_LEGACY_BACKEND
        return std::unique_ptr<SpmpcSolver>(new ContinuousMpccDirectOmegaLegacySolverAcados());
#else
        throw std::invalid_argument(
            "SPMPC legacy solver backend is disabled at build time: " +
            backend);
#endif
    }
    if (backend == kSolverBackendPrimitive) {
        // primitive 是显式 fallback/debug rollout sampler；调用方必须先完成 backend policy 校验。
        return std::unique_ptr<SpmpcSolver>(new RolloutSamplingSolver());
    }
    throw std::invalid_argument("unknown SPMPC solver backend: " + backend);
}

}  // namespace spmpc_local_planner
