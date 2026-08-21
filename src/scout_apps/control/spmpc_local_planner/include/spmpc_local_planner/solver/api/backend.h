#pragma once

#include <string>

namespace spmpc_local_planner {

constexpr const char* kSolverBackendPrimitive = "primitive";
constexpr const char* kSolverBackendContinuousMpccAcados =
    "continuous_mpcc_acados";
constexpr const char* kSolverBackendContinuousMpccDirectOmegaLegacy =
    "continuous_mpcc_direct_omega_legacy";
constexpr const char* kSolverBackendDelayAugmentedPhaseAcados =
    "delay_augmented_phase_acados";

bool isKnownSolverBackend(const std::string& backend);
const char* solverBackendRole(const std::string& backend);
int solverBackendCode(const std::string& backend);

}  // namespace spmpc_local_planner
