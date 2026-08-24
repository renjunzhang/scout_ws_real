#include "spmpc_local_planner/solvers/solver_factory.h"
#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"
#include "spmpc_local_planner/solvers/delay_augmented_phase_online_solver.h"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

namespace spmpc_local_planner {
namespace {

TEST(SolverFactory, DefaultConfigStillConstructsOriginalMainlineBackend) {
    const SolverParams defaults;
    EXPECT_EQ(defaults.solver_backend,
              kSolverBackendContinuousMpccAcados);
    EXPECT_FALSE(defaults.delay_augmented_phase.enabled);
    std::unique_ptr<SpmpcSolver> mainline =
        makeSolver(defaults.solver_backend);
    EXPECT_NE(dynamic_cast<ContinuousMpccSolverAcados*>(mainline.get()),
              nullptr);
    EXPECT_NE(makeSolver(kSolverBackendPrimitive), nullptr);
}

TEST(SolverFactory, ExplicitAugmentedBackendConstructsOnlyTheManifestAdapter) {
    std::unique_ptr<SpmpcSolver> solver =
        makeSolver(kSolverBackendDelayAugmentedPhaseAcados);
    EXPECT_NE(
        dynamic_cast<DelayAugmentedPhaseOnlineSolver*>(solver.get()),
        nullptr);
    EXPECT_EQ(dynamic_cast<ContinuousMpccSolverAcados*>(solver.get()),
              nullptr);
}

TEST(SolverFactory, UnknownBackendFailsClosed) {
    EXPECT_THROW(makeSolver("continuous_mpcc_typo"), std::invalid_argument);
}

TEST(SolverFactory, LegacyBackendMatchesBuildCapability) {
#ifdef SPMPC_BUILD_LEGACY_BACKEND
    EXPECT_NE(
        makeSolver(kSolverBackendContinuousMpccDirectOmegaLegacy), nullptr);
#else
    try {
        makeSolver(kSolverBackendContinuousMpccDirectOmegaLegacy);
        FAIL() << "disabled legacy backend unexpectedly constructed";
    } catch (const std::invalid_argument& error) {
        EXPECT_NE(std::string(error.what()).find("disabled at build time"),
                  std::string::npos);
    }
#endif
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
