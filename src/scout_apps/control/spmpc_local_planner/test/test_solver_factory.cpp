#include "spmpc_local_planner/solvers/solver_factory.h"

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

namespace spmpc_local_planner {
namespace {

TEST(SolverFactory, MainlineAndPrimitiveRemainAvailable) {
    EXPECT_NE(makeSolver(kSolverBackendContinuousMpccAcados), nullptr);
    EXPECT_NE(makeSolver(kSolverBackendPrimitive), nullptr);
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
