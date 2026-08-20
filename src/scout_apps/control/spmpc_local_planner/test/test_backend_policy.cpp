#include "spmpc_local_planner/solver/api/backend.h"
#include "spmpc_local_planner/solver/api/backend_policy.h"

#include <gtest/gtest.h>

#include <string>

namespace spmpc_local_planner {
namespace {

TEST(BackendPolicy, IdentifiesStableBackendRolesAndCodes) {
    EXPECT_TRUE(isKnownSolverBackend(kSolverBackendContinuousMpccAcados));
    EXPECT_TRUE(isKnownSolverBackend(
        kSolverBackendContinuousMpccDirectOmegaLegacy));
    EXPECT_TRUE(isKnownSolverBackend(kSolverBackendPrimitive));
    EXPECT_FALSE(isKnownSolverBackend("typo"));

    EXPECT_EQ(solverBackendCode(kSolverBackendContinuousMpccAcados), 1);
    EXPECT_EQ(
        solverBackendCode(kSolverBackendContinuousMpccDirectOmegaLegacy), 2);
    EXPECT_EQ(solverBackendCode(kSolverBackendPrimitive), 3);
    EXPECT_EQ(solverBackendCode("typo"), 0);
    EXPECT_STREQ(solverBackendRole("typo"), "unknown");
}

TEST(BackendPolicy, MainlineAllowsSupportedSloshVariant) {
    SolverParams params;
    params.solver_backend = kSolverBackendContinuousMpccAcados;
    VariantConfig variant;
    variant.slosh_enable = true;
    variant.slosh_constraint_enable = true;

    std::string reason;
    EXPECT_TRUE(validateBackendPolicy(params, variant, reason));
    EXPECT_TRUE(reason.empty());
}

TEST(BackendPolicy, MainlineAccumulatesEveryUnsupportedFeature) {
    SolverParams params;
    params.solver_backend = kSolverBackendContinuousMpccAcados;
    params.corridor_enable = true;
    params.obstacle_enable = true;
    params.homotopy_enable = true;
    params.corridor_hard_bound_enable = true;
    VariantConfig variant;
    variant.slosh_constraint_enable = true;

    std::string reason;
    EXPECT_FALSE(validateBackendPolicy(params, variant, reason));
    EXPECT_NE(reason.find("slosh_constraint_enable requires"), std::string::npos);
    EXPECT_NE(reason.find("corridor_enable"), std::string::npos);
    EXPECT_NE(reason.find("obstacle_enable"), std::string::npos);
    EXPECT_NE(reason.find("homotopy_enable"), std::string::npos);
    EXPECT_NE(reason.find("corridor_hard_bound_enable"), std::string::npos);
}

TEST(BackendPolicy, LegacyAndPrimitiveRejectSloshMainlineVariants) {
    VariantConfig variant;
    variant.slosh_enable = true;
    std::string reason;

    SolverParams legacy;
    legacy.solver_backend = kSolverBackendContinuousMpccDirectOmegaLegacy;
    EXPECT_FALSE(validateBackendPolicy(legacy, variant, reason));
    EXPECT_NE(reason.find("RouteB legacy"), std::string::npos);

    SolverParams primitive;
    primitive.solver_backend = kSolverBackendPrimitive;
    EXPECT_FALSE(validateBackendPolicy(primitive, variant, reason));
    EXPECT_NE(reason.find("fallback/debug"), std::string::npos);
}

TEST(BackendPolicy, UnknownBackendFailsClosed) {
    SolverParams params;
    params.solver_backend = "continuous_mpcc_typo";
    std::string reason;
    EXPECT_FALSE(validateBackendPolicy(params, VariantConfig{}, reason));
    EXPECT_EQ(reason, "unknown solver backend");
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
