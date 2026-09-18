#include "spmpc_local_planner/ros/control_cycle_contract.h"
#include "spmpc_local_planner/ros/odom_state_buffer.h"
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

TEST(OdomStateBuffer, ObserverAdvancingBeforeOdomCommitCannotSplitControlEpochs) {
    SloshObserverBank observer;
    ASSERT_TRUE(observer.configure(SloshModelParams{}, .02));
    MotionExcitation excitation;
    excitation.valid = true; excitation.source = MotionExcitationSource::Odom;
    excitation.sample_dt_sec = .02; excitation.measurement_stamp_ns = 42925000000LL;
    ASSERT_TRUE(observer.stepOdom(excitation));
    OdomStateBuffer buffer;
    const auto previous = sample(excitation.measurement_stamp_ns, 1., .1, .55, -.1);
    ASSERT_TRUE(buffer.commit(previous, observer.odom(), "odom", previous.stamp_ns+1000000, 1.));

    // Reproduce the recorded scheduling window: the observer has processed
    // 42.945 s but the odom worker has not committed that robot/history yet.
    excitation.measurement_stamp_ns += 20000000LL;
    excitation.ax = .1;
    ASSERT_TRUE(observer.stepOdom(excitation));
    const auto during_update = buffer.snapshot();
    EXPECT_EQ(alignRobotStateToEpoch(during_update.history,
        observer.odom().state_stamp_ns, .05, .01).status, "EXTRAPOLATION_LIMIT");
    const auto coherent = alignRobotStateToEpoch(during_update.history,
        during_update.liquid.state_stamp_ns, .05, .01);
    ASSERT_TRUE(coherent.valid); EXPECT_EQ(coherent.status, "EXACT");
    EXPECT_EQ(during_update.robot.stamp_ns, during_update.liquid.state_stamp_ns);
    EXPECT_DOUBLE_EQ(coherent.state.v, .55);

    const auto current = sample(excitation.measurement_stamp_ns, 1.011, .098, .552, -.1);
    ASSERT_TRUE(buffer.commit(current, observer.odom(), "odom", current.stamp_ns+1000000, 1.));
    const auto after = buffer.snapshot();
    EXPECT_EQ(after.robot.stamp_ns, after.liquid.state_stamp_ns);
    EXPECT_TRUE(alignRobotStateToEpoch(after.history, after.liquid.state_stamp_ns, .05, .01).valid);
    EXPECT_EQ(during_update.history.back().stamp_ns, previous.stamp_ns);
    EXPECT_EQ(during_update.liquid.state_stamp_ns, previous.stamp_ns);
    EXPECT_EQ(after.history.back().stamp_ns, current.stamp_ns);
}

TEST(OdomStateBuffer, RejectsMismatchedValidPairWithoutReplacingAcceptedState) {
    OdomStateBuffer buffer;
    SloshObserverSnapshot liquid; liquid.configured = liquid.valid = true;
    liquid.state_stamp_ns = 1000000000LL;
    ASSERT_TRUE(buffer.commit(sample(liquid.state_stamp_ns, 0,0,0,0), liquid, "odom", 1001000000LL, 1.));
    liquid.state_stamp_ns += 20000000LL;
    EXPECT_FALSE(buffer.commit(sample(1000000000LL, 0,0,0,0), liquid, "odom", 1021000000LL, 1.));
    EXPECT_EQ(buffer.snapshot().liquid.state_stamp_ns, 1000000000LL);
    EXPECT_EQ(buffer.snapshot().robot.stamp_ns, 1000000000LL);
}

TEST(OdomStateBuffer, ClockResetRetainsLiquidInvalidationAndClearsOldHistory) {
    OdomStateBuffer buffer;
    SloshObserverSnapshot liquid; liquid.configured = liquid.valid = true;
    for (int k=0; k<10; ++k) {
        liquid.state_stamp_ns = 1000000000LL+k*20000000LL;
        ASSERT_TRUE(buffer.commit(sample(liquid.state_stamp_ns, 0,0,0,0), liquid,
                                 "odom", liquid.state_stamp_ns, .1));
    }
    EXPECT_LE(buffer.snapshot().history.size(), 6u);
    liquid.valid = false; liquid.state.eta_x = .003;
    ASSERT_TRUE(buffer.commit(sample(500000000LL, 0,0,0,0), liquid, "odom", 500000000LL, .1));
    const auto reset = buffer.snapshot();
    ASSERT_EQ(reset.history.size(), 1u);
    EXPECT_EQ(reset.robot.stamp_ns, 500000000LL);
    EXPECT_FALSE(reset.liquid.valid);
    EXPECT_DOUBLE_EQ(reset.liquid.state.eta_x, .003);
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
    const auto result = propagateReferencePoseToEpoch(pose, history, 1010000000LL, .05, .01, .01);
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
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1010000001LL, .05, .01, .01).valid);
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 999000000LL, .05, .01, .01).valid);
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1001000000LL, .05, 0., 0.).valid);
    history.pop_front();
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1005000000LL, .05, .01, .01).valid);
}

TEST(ControlCycleContract, MeasuredOdomBridgeDoesNotUseRobotExtrapolationAllowance) {
    // The recorded failures had a 20/21 ms old localization anchor while both
    // odom endpoints were already measured. No constant-twist prediction is needed.
    std::deque<StampedRobotState> history{
        sample(1000000000LL, 1., 0., .2, .1),
        sample(1020000000LL, 1.006, .004, .4, .2),
        sample(1040000000LL, 1.014, .008, .5, .2),
    };
    const auto pose = sample(1000000000LL, 10., M_PI/2., 0., 0.);
    const auto measured = propagateReferencePoseToEpoch(pose, history, 1020000000LL, .05, 0., .05);
    ASSERT_TRUE(measured.valid);
    EXPECT_NEAR(measured.state.x, 10., 1e-12);
    EXPECT_NEAR(measured.state.y, .006, 1e-12);
    EXPECT_NEAR(measured.state.yaw, M_PI/2.+.004, 1e-12);
    EXPECT_DOUBLE_EQ(measured.state.v, .4);
    const auto interpolated = propagateReferencePoseToEpoch(pose, history, 1021000000LL, .05, 0., .05);
    ASSERT_TRUE(interpolated.valid);
    EXPECT_TRUE(interpolated.interpolated);
    EXPECT_NEAR(interpolated.state.y, .0064, 1e-12);
    // A wider localization age does not allow missing odom, a wider sample gap,
    // or motion beyond the unchanged 10 ms unmeasured extrapolation bound.
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1051000000LL, .05, .01, .10).valid);
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1021000000LL, .01, .01, .05).valid);
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1020000000LL, .05, .01, .01).valid);
    history.push_back(sample(1060000000LL, 1.02, .012, .5, .2));
    EXPECT_FALSE(propagateReferencePoseToEpoch(pose, history, 1050000001LL, .05, .01, .05).valid);
}

TEST(ControlCycleContract, LocalizationPropagationInterpolatesAndWrapsYaw) {
    std::deque<StampedRobotState> history{
        sample(1000000000LL, 0., M_PI-.01, .2, .5),
        sample(1010000000LL, .002, -M_PI+.01, .4, .5),
    };
    const auto pose = sample(1002000000LL, 5., M_PI-.005, 0., 0.);
    const auto result = propagateReferencePoseToEpoch(pose, history, 1008000000LL, .05, .01, .01);
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
