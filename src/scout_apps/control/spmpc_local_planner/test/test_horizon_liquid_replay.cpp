#include "spmpc_local_planner/analysis/horizon_liquid_replay.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace replay = spmpc_local_planner::analysis;

namespace {

void expectStateNear(const replay::ModalState& actual,
                     const replay::ModalState& expected,
                     double tolerance = 1e-12) {
    EXPECT_NEAR(actual.eta_x, expected.eta_x, tolerance);
    EXPECT_NEAR(actual.eta_x_dot, expected.eta_x_dot, tolerance);
    EXPECT_NEAR(actual.eta_y, expected.eta_y, tolerance);
    EXPECT_NEAR(actual.eta_y_dot, expected.eta_y_dot, tolerance);
}

TEST(HorizonLiquidReplay, HermitePreservesBothEndpointStates) {
    replay::ModalState left{0.3, -0.7, 1.2, 0.4};
    replay::ModalState right{-0.8, 0.9, 0.2, -1.1};
    const auto at_left = replay::cubicHermite(left, right, 0.0, 0.37);
    const auto at_right = replay::cubicHermite(left, right, 0.37, 0.37);
    ASSERT_TRUE(at_left.success) << at_left.detail;
    ASSERT_TRUE(at_right.success) << at_right.detail;
    expectStateNear(at_left.state, left);
    expectStateNear(at_right.state, right);
}

TEST(HorizonLiquidReplay, ExactZohMatchesUndampedAnalyticSolution) {
    replay::ModalParameters parameters{0.0, 4.0, 1.5, 0.75};
    replay::ModalState initial{0.3, -0.4, -0.2, 0.5};
    const double duration = 0.37;
    const auto actual = replay::exactZohForcedModalStep(
        initial, 0.8, -0.6, duration, parameters);
    ASSERT_TRUE(actual.success) << actual.detail;
    const auto analytic = [&](double position, double velocity,
                              double acceleration, double gain) {
        replay::ModalState axis;
        const double equilibrium = -gain * acceleration / 4.0;
        const double shifted = position - equilibrium;
        axis.eta_x = equilibrium + shifted * std::cos(2.0 * duration) +
                     velocity * std::sin(2.0 * duration) / 2.0;
        axis.eta_x_dot = -2.0 * shifted * std::sin(2.0 * duration) +
                         velocity * std::cos(2.0 * duration);
        return axis;
    };
    const replay::ModalState x = analytic(0.3, -0.4, 0.8, 1.5);
    const replay::ModalState y = analytic(-0.2, 0.5, -0.6, 0.75);
    EXPECT_NEAR(actual.state.eta_x, x.eta_x, 1e-12);
    EXPECT_NEAR(actual.state.eta_x_dot, x.eta_x_dot, 1e-12);
    EXPECT_NEAR(actual.state.eta_y, y.eta_x, 1e-12);
    EXPECT_NEAR(actual.state.eta_y_dot, y.eta_x_dot, 1e-12);
}

TEST(HorizonLiquidReplay, ExactZohSupportsCriticalAndOverdampedModes) {
    const replay::ModalState initial{0.3, -0.4, -0.2, 0.5};
    const std::vector<replay::ModalParameters> parameter_sets = {
        {4.0, 4.0, 1.5, 0.75},
        {8.0, 4.0, 1.5, 0.75},
    };
    for (const replay::ModalParameters& parameters : parameter_sets) {
        const auto whole = replay::exactZohForcedModalStep(
            initial, 0.8, -0.6, 0.37, parameters);
        ASSERT_TRUE(whole.success) << whole.detail;
        replay::ModalState repeated = initial;
        for (int step_index = 0; step_index < 37; ++step_index) {
            const auto step = replay::exactZohForcedModalStep(
                repeated, 0.8, -0.6, 0.01, parameters);
            ASSERT_TRUE(step.success) << step.detail;
            repeated = step.state;
        }
        expectStateNear(whole.state, repeated, 2e-14);
    }
}

TEST(HorizonLiquidReplay, ObserverAnchorEchoIsSkippedExactlyOnce) {
    replay::ModalParameters parameters{0.4, 9.0, 1.0, 1.0};
    replay::ObserverAnchor anchor;
    anchor.state_stamp_ns = 1000000000;
    anchor.update_count = 7;
    anchor.reset_epoch = 3;
    anchor.state = replay::ModalState{0.1, -0.2, 0.3, -0.4};
    replay::ObserverInputSample echo;
    echo.state_stamp_ns = anchor.state_stamp_ns;
    echo.sample_dt_sec = 0.01;
    echo.update_count = anchor.update_count;
    echo.reset_epoch = anchor.reset_epoch;
    echo.ax = 100.0;
    echo.ay = -100.0;
    replay::ObserverInputSample following;
    following.state_stamp_ns = 1010000000;
    following.sample_dt_sec = 0.01;
    following.update_count = 8;
    following.reset_epoch = 3;
    following.ax = 0.6;
    following.ay = -0.2;
    const auto with_echo = replay::replayObserverInputs(
        anchor, {echo, following}, parameters);
    const auto without_echo = replay::replayObserverInputs(
        anchor, {following}, parameters);
    ASSERT_TRUE(with_echo.success) << with_echo.detail;
    ASSERT_TRUE(without_echo.success) << without_echo.detail;
    EXPECT_TRUE(with_echo.skipped_anchor_echo);
    ASSERT_EQ(with_echo.points.size(), 2u);
    expectStateNear(with_echo.points.back().state,
                    without_echo.points.back().state);
    const auto duplicate = replay::replayObserverInputs(
        anchor, {echo, echo, following}, parameters);
    EXPECT_FALSE(duplicate.success);
}

TEST(HorizonLiquidReplay, ObserverEpochResetAndInteriorSamplingMatchZoh) {
    const replay::ModalParameters parameters{0.4, 9.0, 1.0, 1.0};
    replay::ObserverAnchor anchor;
    anchor.state_stamp_ns = 1000000000;
    anchor.update_count = 7;
    anchor.reset_epoch = 3;
    anchor.state = replay::ModalState{0.1, -0.2, 0.3, -0.4};
    replay::ObserverInputSample reset;
    reset.state_stamp_ns = 1050000000;
    reset.sample_dt_sec = 0.01;
    reset.update_count = 1;
    reset.reset_epoch = 4;
    reset.ax = 0.6;
    reset.ay = -0.2;
    replay::ObserverInputSample following;
    following.state_stamp_ns = 1060000000;
    following.sample_dt_sec = 0.01;
    following.update_count = 2;
    following.reset_epoch = 4;
    following.ax = -0.3;
    following.ay = 0.5;
    const auto replayed = replay::replayObserverInputs(
        anchor, {reset, following}, parameters, 1.0e-7, true);
    ASSERT_TRUE(replayed.success) << replayed.detail;
    ASSERT_EQ(replayed.epoch_reset_count, 1u);
    ASSERT_EQ(replayed.points.size(), 3u);
    EXPECT_TRUE(replayed.points[1].epoch_reset_applied);
    const auto reset_expected = replay::exactZohForcedModalStep(
        {}, reset.ax, reset.ay, reset.sample_dt_sec, parameters);
    ASSERT_TRUE(reset_expected.success) << reset_expected.detail;
    expectStateNear(replayed.points[1].state, reset_expected.state);

    const auto sampled = replay::sampleObserverReplay(
        replayed.points, 1055000000, parameters);
    ASSERT_TRUE(sampled.success) << sampled.detail;
    const auto sample_expected = replay::exactZohForcedModalStep(
        replayed.points[1].state, following.ax, following.ay,
        0.005, parameters);
    ASSERT_TRUE(sample_expected.success) << sample_expected.detail;
    expectStateNear(sampled.state, sample_expected.state);
}

TEST(HorizonLiquidReplay, PlannedRk4UsesSharedNominalDynamics) {
    replay::ModalParameters parameters{0.6, 6.25, 0.9, 1.1};
    replay::ModalState initial{0.12, -0.08, 0.0, 0.0};
    std::vector<replay::PlannedControl> controls = {
        {0.8, 0.0, 0.07}, {-0.3, 0.0, 0.05},
    };
    const auto planned = replay::replayPlannedControls(
        initial, 0.4, 0.0, controls, parameters, 5.0e-4);
    ASSERT_TRUE(planned.success) << planned.detail;
    replay::ModalState expected = initial;
    double elapsed = 0.0;
    for (const auto& control : controls) {
        const auto step = replay::exactZohForcedModalStep(
            expected, control.a, 0.0, control.duration_sec, parameters);
        ASSERT_TRUE(step.success) << step.detail;
        expected = step.state;
        elapsed += control.duration_sec;
        const auto sampled = replay::samplePlannedReplay(
            planned.points, elapsed);
        ASSERT_TRUE(sampled.success) << sampled.detail;
        EXPECT_NEAR(sampled.point.state.eta_x, expected.eta_x, 1e-10);
        EXPECT_NEAR(sampled.point.state.eta_x_dot,
                    expected.eta_x_dot, 1e-10);
    }
}

TEST(HorizonLiquidReplay, EmptyPlannedControlsPreserveInitialPoint) {
    const replay::ModalParameters parameters{0.6, 6.25, 0.9, 1.1};
    const replay::ModalState initial{0.12, -0.08, 0.03, 0.09};
    const auto planned = replay::replayPlannedControls(
        initial, 0.4, -0.2, {}, parameters);
    ASSERT_TRUE(planned.success) << planned.detail;
    ASSERT_EQ(planned.points.size(), 1u);
    EXPECT_DOUBLE_EQ(planned.points.front().time_sec, 0.0);
    EXPECT_DOUBLE_EQ(planned.points.front().v, 0.4);
    EXPECT_DOUBLE_EQ(planned.points.front().omega, -0.2);
    EXPECT_EQ(planned.points.front().control_index, -1);
    expectStateNear(planned.points.front().state, initial);
    const auto sampled = replay::samplePlannedReplay(planned.points, 0.0);
    ASSERT_TRUE(sampled.success) << sampled.detail;
    expectStateNear(sampled.point.state, initial);
}

TEST(HorizonLiquidReplay, ContractsFailClosedOnOrderAndRangeErrors) {
    replay::ModalParameters parameters{0.4, 9.0, 1.0, 1.0};
    replay::ObserverAnchor anchor;
    anchor.state_stamp_ns = 1000000000;
    anchor.update_count = 7;
    anchor.reset_epoch = 3;
    replay::ObserverInputSample count_gap;
    count_gap.state_stamp_ns = 1010000000;
    count_gap.sample_dt_sec = 0.01;
    count_gap.update_count = 9;
    count_gap.reset_epoch = 3;
    EXPECT_FALSE(replay::replayObserverInputs(
        anchor, {count_gap}, parameters).success);
    EXPECT_FALSE(replay::cubicHermite({}, {}, 0.2, 0.1).success);
    EXPECT_FALSE(replay::samplePlannedReplay({}, 0.0).success);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
