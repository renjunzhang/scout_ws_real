#include "spmpc_local_planner/runtime/state_alignment.h"

#include <gtest/gtest.h>

#include <deque>

namespace spmpc_local_planner {
namespace {

StampedRobotState sample(std::int64_t stamp_ns,
                         double x,
                         double yaw,
                         double v,
                         double omega) {
    StampedRobotState out;
    out.stamp_ns = stamp_ns;
    out.state.x = x;
    out.state.yaw = yaw;
    out.state.v = v;
    out.state.omega = omega;
    return out;
}

TEST(ControlCycleContract, InterpolatesRobotAtLiquidEpoch) {
    std::deque<StampedRobotState> history{
        sample(1000000000LL, 0.0, 0.0, 0.2, 0.1),
        sample(1020000000LL, 0.004, 0.002, 0.2, 0.1),
    };
    const auto result = alignRobotStateToEpoch(
        history, 1010000000LL, 0.05, 0.01);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.interpolated);
    EXPECT_FALSE(result.extrapolated);
    EXPECT_NEAR(result.state.x, 0.002, 1e-12);
    EXPECT_NEAR(result.state.yaw, 0.001, 1e-12);
}

TEST(ControlCycleContract, RejectsGapAndExtrapolationBeyondLimits) {
    std::deque<StampedRobotState> history{
        sample(1000000000LL, 0.0, 0.0, 0.2, 0.0),
        sample(1100000000LL, 0.02, 0.0, 0.2, 0.0),
    };
    EXPECT_FALSE(alignRobotStateToEpoch(
        history, 1050000000LL, 0.05, 0.01).valid);
    EXPECT_FALSE(alignRobotStateToEpoch(
        history, 1120000000LL, 0.20, 0.01).valid);
}

TEST(ControlCycleContract, AllowsOnlyBoundedRawSkew) {
    double skew = 0.0;
    EXPECT_TRUE(stateSkewWithinContract(
        1057000000LL, 1000000000LL, 0.080, skew));
    EXPECT_NEAR(skew, 0.057, 1e-12);
    EXPECT_FALSE(stateSkewWithinContract(
        1090000000LL, 1000000000LL, 0.080, skew));
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
