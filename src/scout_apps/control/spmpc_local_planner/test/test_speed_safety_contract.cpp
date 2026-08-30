#include "spmpc_local_planner/core/speed_safety_contract.h"

#include <gtest/gtest.h>
#include <limits>
#include <string>

namespace spmpc_local_planner {
namespace {

TEST(SpeedSafetyContractTest, DisabledContractPreservesPlatformLimit) {
    SpeedSafetyContract contract;
    SpeedSafetyParams params;
    params.enable = false;
    params.v_safe_max = 0.15;

    std::string error;
    ASSERT_TRUE(contract.configure(params, 0.8, &error)) << error;
    EXPECT_DOUBLE_EQ(contract.effectiveVMax(), 0.8);

    const auto decision = contract.inspect(0.6, 0.6, 0.6);
    EXPECT_FALSE(decision.enabled);
    EXPECT_FALSE(decision.violation);
    EXPECT_FALSE(decision.latched);
}

TEST(SpeedSafetyContractTest, EnabledContractCapsSolverAtSafetyLimit) {
    SpeedSafetyContract contract;
    SpeedSafetyParams params;
    params.enable = true;
    params.v_safe_max = 0.15;
    params.tolerance = 1e-4;

    std::string error;
    ASSERT_TRUE(contract.configure(params, 0.8, &error)) << error;
    EXPECT_DOUBLE_EQ(contract.platformVMax(), 0.8);
    EXPECT_DOUBLE_EQ(contract.effectiveVMax(), 0.15);

    const auto decision = contract.inspect(0.10, 0.10, 0.10);
    EXPECT_EQ(decision.status, "PASS");
    EXPECT_FALSE(decision.violation);
    EXPECT_FALSE(decision.latched);
}

TEST(SpeedSafetyContractTest, RejectsInvalidEnabledConfiguration) {
    SpeedSafetyContract contract;
    SpeedSafetyParams params;
    params.enable = true;
    params.v_safe_max = 0.9;

    std::string error;
    EXPECT_FALSE(contract.configure(params, 0.8, &error));
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(contract.configured());

    params.v_safe_max = 0.15;
    params.tolerance = -1.0;
    EXPECT_FALSE(contract.configure(params, 0.8, &error));
}

TEST(SpeedSafetyContractTest, SolverViolationLatches) {
    SpeedSafetyContract contract;
    SpeedSafetyParams params;
    params.enable = true;
    ASSERT_TRUE(contract.configure(params, 0.8));

    const auto first = contract.inspect(0.16, 0.10, 0.10);
    EXPECT_TRUE(first.solver_violation);
    EXPECT_TRUE(first.violation);
    EXPECT_TRUE(first.newly_latched);
    EXPECT_TRUE(first.latched);

    const auto second = contract.inspect(0.10, 0.10, 0.10);
    EXPECT_FALSE(second.violation);
    EXPECT_FALSE(second.newly_latched);
    EXPECT_TRUE(second.latched);
    EXPECT_EQ(second.status, "LATCHED");
}

TEST(SpeedSafetyContractTest, ChecksPostGateAndPublishCandidateIndependently) {
    SpeedSafetyContract contract;
    SpeedSafetyParams params;
    params.enable = true;
    ASSERT_TRUE(contract.configure(params, 0.8));

    auto decision = contract.inspect(0.10, -0.16, 0.10);
    EXPECT_TRUE(decision.post_gate_violation);
    EXPECT_FALSE(decision.publish_candidate_violation);

    contract.reset();
    decision = contract.inspect(0.10, 0.10, -0.16);
    EXPECT_FALSE(decision.post_gate_violation);
    EXPECT_TRUE(decision.publish_candidate_violation);
}

TEST(SpeedSafetyContractTest, ToleranceBoundaryPassesAndNonFiniteFails) {
    SpeedSafetyContract contract;
    SpeedSafetyParams params;
    params.enable = true;
    params.v_safe_max = 0.15;
    params.tolerance = 1e-4;
    ASSERT_TRUE(contract.configure(params, 0.8));

    auto decision = contract.inspect(0.15009, -0.15009, 0.15009);
    EXPECT_FALSE(decision.violation);

    decision = contract.inspect(
        std::numeric_limits<double>::quiet_NaN(), 0.10, 0.10);
    EXPECT_TRUE(decision.solver_violation);
    EXPECT_TRUE(decision.latched);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
