#include "spmpc_local_planner/ros/control_cycle_contract.h"
#include "spmpc_local_planner/reference/progress_projector.h"

#include <gtest/gtest.h>

#include <deque>
#include <cmath>

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

TEST(ControlCycleContract, VehicleEpochDoesNotBorrowANewerPoseOrTwist) {
    // New odometry may arrive while the control callback resolves TF. The
    // copied cycle stamp still identifies the same pair in the live history.
    std::deque<StampedRobotState> history{
        sample(1000000000LL, 1.0, .1, .2, .1),
        sample(1100000000LL, 1.025, .12, .3, .2),
    };
    const auto result=alignRobotStateToEpoch(history,1000000000LL,.05,.01);
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.status,"EXACT");
    EXPECT_DOUBLE_EQ(result.state.x,1.0);
    EXPECT_DOUBLE_EQ(result.state.v,.2);
    EXPECT_DOUBLE_EQ(result.state.omega,.1);
    EXPECT_FALSE(result.interpolated);
    EXPECT_FALSE(result.extrapolated);
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

TEST(ControlCycleContract, PropagatesDelayedLocalizationUsingStampedOdomMotion) {
    // Odom drives along +x while the localization frame is rotated +90 deg.
    // Include a newer sample to catch accidentally borrowing the latest twist.
    std::deque<StampedRobotState> history{
        sample(1000000000LL, 1.0, 0.0, 0.2, 0.1),
        sample(1010000000LL, 1.003, 0.002, 0.4, 0.2),
        sample(1020000000LL, 1.009, 0.007, 0.8, 0.5),
    };
    auto pose = sample(1000000000LL, 10., M_PI/2., 0., 0.);
    pose.state.y = 20.;
    const auto result = propagateReferencePoseToEpoch(pose, history, 1010000000LL, .05, .01);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.extrapolated);
    EXPECT_NEAR(result.state.x, 10., 1e-12);
    EXPECT_NEAR(result.state.y, 20.003, 1e-12);
    EXPECT_NEAR(result.state.yaw, M_PI/2.+.002, 1e-12);
    EXPECT_DOUBLE_EQ(result.state.v, .4);
    EXPECT_DOUBLE_EQ(result.state.omega, .2);
}

TEST(ControlCycleContract, LocalizationPropagationRejectsOldFutureAndMissingAnchors) {
    std::deque<StampedRobotState> history{
        sample(1000000000LL, 0., 0., .2, 0.),
        sample(1020000000LL, .004, 0., .2, 0.),
    };
    const auto pose = sample(1000000000LL, 10., .1, 0., 0.);
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1010000001LL, .05, .01).valid);
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 999000000LL, .05, .01).valid);
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1001000000LL, .05, 0.).valid);
    history.pop_front();
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1005000000LL, .05, .01).valid);
}

TEST(ControlCycleContract, LocalizationPropagationInterpolatesAndWrapsYaw) {
    std::deque<StampedRobotState> history{
        sample(1000000000LL, 0., M_PI-.01, .2, .5),
        sample(1010000000LL, .002, -M_PI+.01, .4, .5),
    };
    const auto pose = sample(1002000000LL, 5., M_PI-.005, 0., 0.);
    const auto result = propagateReferencePoseToEpoch(pose, history, 1008000000LL, .05, .01);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(result.interpolated);
    EXPECT_NEAR(result.state.yaw, -M_PI+.007, 1e-12);
    EXPECT_NEAR(result.state.v, .36, 1e-12);
}

TEST(ControlCycleContract, RefusesResultThatExpiredDuringSolveOrPostProcessing) {
    ControlCycleTimingDebug timing;
    timing.cycle_start_stamp_ns = 1000000000LL;
    timing.solver_input_epoch_ns = 1002000000LL;
    EXPECT_TRUE(commandResultFresh(timing, 1030000000LL, 1.0/30.0));
    EXPECT_FALSE(commandResultFresh(timing, 1082000000LL, 1.0/30.0));
    EXPECT_FALSE(commandResultFresh(timing, 1001000000LL, 1.0/30.0));
    timing.solver_input_epoch_ns = 990000000LL;
    EXPECT_FALSE(commandResultFresh(timing, 1030000000LL, 1.0/30.0));
}

TEST(ControlCycleContract, RemainingBudgetStopsFurtherIterations) {
    const auto start = SolveBudget::Clock::now();
    SolveBudget budget;
    EXPECT_TRUE(budget.permits(1.0, start));  // offline, no real-time deadline
    budget.deadline = start + std::chrono::milliseconds(27);
    EXPECT_TRUE(budget.permits(.006, start + std::chrono::milliseconds(20)));
    EXPECT_FALSE(budget.permits(.008, start + std::chrono::milliseconds(20)));
    EXPECT_FALSE(budget.permits(0.0, start + std::chrono::milliseconds(28)));
}

TEST(ControlCycleContract, LocalizationOutlierDoesNotAdvancePersistentProgress) {
    PoseContinuityGuard guard;
    PoseContinuityParams params;
    ReferencePath path;
    path.setPoints({{0,0,0,0,0},{3,0,0,0,0}}, "map");
    ProgressProjector projector;
    ProgressProjectionState progress;
    const auto observe = [&](std::int64_t time, double x) {
        const auto current = sample(time,x,0.,.25,0.);
        if (guard.observe(current,params)) projector.project(path,x,0.,progress);
    };
    observe(1000000000LL,1.0);
    observe(1000000000LL,1.4);  // TF can be revised at the same odometry epoch.
    EXPECT_DOUBLE_EQ(progress.progress,1.0);
    observe(1020000000LL,1.4);
    EXPECT_DOUBLE_EQ(progress.progress,1.0);
    observe(1040000000LL,1.01);
    EXPECT_NEAR(progress.progress,1.01,1e-12);
}

TEST(ControlCycleContract, PoseGuardAcceptsPhysicalTurnsAwayFromRoute) {
    PoseContinuityGuard guard;
    PoseContinuityParams params;
    auto first = sample(1000000000LL,0.,0.,.5,1.);
    ASSERT_TRUE(guard.observe(first,params));
    for (int k=1;k<=20;++k) {
        const double t=.02*k;
        auto current=sample(1000000000LL+20000000LL*k,.5*std::sin(t),t,.5,1.);
        current.state.y=.5*(1-std::cos(t));
        EXPECT_TRUE(guard.observe(current,params));
    }
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
