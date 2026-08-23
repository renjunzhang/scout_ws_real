#include "spmpc_local_planner/core/terminal_controller.h"
#include <gtest/gtest.h>
#include <cmath>

namespace spmpc_local_planner {
namespace {

TerminalControllerParams makeParams() {
    TerminalControllerParams params;
    params.enable = true;
    params.goal_tolerance = 0.15;
    params.slowdown_enable = true;
    params.slowdown_distance = 1.20;
    params.slowdown_v_max = 0.18;
    params.capture_stop_enable = true;
    params.capture_stop_distance = 0.70;
    params.capture_v_cap = 0.16;
    params.goal_behind_x = -0.05;
    params.goal_reached_max_speed = 0.03;
    params.goal_reached_max_omega = 0.05;
    params.command_clamp_enable = true;
    params.rate_limit_enable = true;
    params.omega_clamp_enable = true;
    params.omega_clamp_max = 0.25;
    params.omega_near_goal_max = 0.10;
    params.omega_near_goal_distance = 0.35;
    return params;
}

TerminalGoalInfo makeGoal(double distance, double dx_robot = 1.0, double remaining_s = -1.0) {
    TerminalGoalInfo goal;
    goal.valid = true;
    goal.distance_to_goal = distance;
    goal.remaining_s = remaining_s >= 0.0 ? remaining_s : distance;
    goal.dx_robot = dx_robot;
    goal.position_reached = distance < 0.15;
    return goal;
}

}  // namespace

TEST(TerminalController, FarFromGoalDoesNotEnterTerminalPhase) {
    TerminalController controller;
    controller.setParams(makeParams());
    const auto plan = controller.updateAndPlan(makeGoal(2.0), 0.3, 0.0, 0.6);
    EXPECT_FALSE(plan.terminal_phase);
    EXPECT_TRUE(plan.pre_terminal_phase);
    EXPECT_FALSE(plan.envelope_active);
}

TEST(TerminalController, SlowdownEnvelopeActivatesNearGoal) {
    TerminalController controller;
    controller.setParams(makeParams());
    const auto goal = makeGoal(1.0);
    const auto plan = controller.updateAndPlan(goal, 0.3, 0.0, 0.6);
    EXPECT_TRUE(plan.terminal_phase);
    EXPECT_TRUE(plan.envelope_active);
    EXPECT_LT(plan.v_envelope, 0.18 + 1e-9);
    const auto clamp = controller.clampCommand(0.4, 0.1, 0.1, 0.1, goal, plan, 0.6);
    EXPECT_LE(clamp.cmd_v_post, 0.4);
}

TEST(TerminalController, CaptureStopLatches) {
    TerminalController controller;
    controller.setParams(makeParams());
    const auto plan = controller.updateAndPlan(makeGoal(0.5), 0.2, 0.0, 0.6);
    EXPECT_TRUE(plan.terminal_phase);
    EXPECT_TRUE(plan.stop_pending);
    EXPECT_EQ(plan.mode, "TERMINAL_CAPTURE_STOP");
}

TEST(TerminalController, CaptureStopNeverRelaxesSlowdownEnvelope) {
    TerminalController controller;
    auto params = makeParams();
    params.goal_tolerance = 0.15;
    params.slowdown_distance = 1.20;
    params.slowdown_v_max = 0.18;
    params.capture_stop_distance = 0.70;
    params.capture_v_cap = 0.30;
    controller.setParams(params);

    const auto just_outside_capture = makeGoal(0.71);
    const auto slowdown = controller.updateAndPlan(
        just_outside_capture, 0.2, 0.0, 0.6);
    ASSERT_FALSE(slowdown.stop_pending);
    ASSERT_TRUE(std::isfinite(slowdown.v_envelope));

    const auto just_inside_capture = makeGoal(0.69);
    const auto capture = controller.updateAndPlan(
        just_inside_capture, 0.2, 0.0, 0.6);
    ASSERT_TRUE(capture.stop_pending);
    const double expected_slowdown_cap = params.slowdown_v_max *
        ((just_inside_capture.distance_to_goal - params.goal_tolerance) /
         (params.slowdown_distance - params.goal_tolerance));
    EXPECT_NEAR(capture.v_envelope, expected_slowdown_cap, 1e-12);
    EXPECT_LT(capture.v_envelope, params.capture_v_cap);
    EXPECT_LT(capture.v_envelope, slowdown.v_envelope);
}

TEST(TerminalController, ReachedRequiresLowSpeedAndLowOmega) {
    TerminalController controller;
    controller.setParams(makeParams());
    EXPECT_FALSE(controller.updateAndPlan(makeGoal(0.05), 0.2, 0.0, 0.6).mode == "REACHED");
    controller.reset();
    EXPECT_FALSE(controller.updateAndPlan(makeGoal(0.05), 0.0, 0.2, 0.6).mode == "REACHED");
    controller.reset();
    EXPECT_EQ(controller.updateAndPlan(makeGoal(0.05), 0.0, 0.0, 0.6).mode, "REACHED");
    EXPECT_TRUE(controller.reached());
}

TEST(TerminalController, HigherLevelTailCanDeferReachedLatch) {
    TerminalController controller;
    controller.setParams(makeParams());
    auto goal = makeGoal(0.05);
    goal.reached_latch_allowed = false;

    const auto deferred = controller.updateAndPlan(goal, 0.0, 0.0, 0.6);
    EXPECT_EQ(deferred.mode, "REACHED_LATCH_DEFERRED");
    EXPECT_FALSE(controller.reached());
    EXPECT_TRUE(controller.diagnostics().position_reached);
    EXPECT_TRUE(controller.diagnostics().reached_latch_blocked);

    goal.reached_latch_allowed = true;
    const auto released = controller.updateAndPlan(goal, 0.0, 0.0, 0.6);
    EXPECT_EQ(released.mode, "REACHED");
    EXPECT_TRUE(controller.reached());
    EXPECT_FALSE(controller.diagnostics().reached_latch_blocked);
}

TEST(TerminalController, ReachedStaysLatchedAfterVelocityNoise) {
    TerminalController controller;
    controller.setParams(makeParams());
    EXPECT_EQ(controller.updateAndPlan(makeGoal(0.05), 0.0, 0.0, 0.6).mode, "REACHED");
    EXPECT_TRUE(controller.reached());

    const auto noisy = controller.updateAndPlan(makeGoal(0.05), 0.10, 0.20, 0.6);
    EXPECT_EQ(noisy.mode, "REACHED");
    EXPECT_TRUE(noisy.terminal_phase);
    EXPECT_TRUE(controller.reached());
}

TEST(TerminalController, ResetClearsReachedLatch) {
    TerminalController controller;
    controller.setParams(makeParams());
    EXPECT_EQ(controller.updateAndPlan(makeGoal(0.05), 0.0, 0.0, 0.6).mode, "REACHED");
    EXPECT_TRUE(controller.reached());

    controller.reset();
    const auto plan = controller.updateAndPlan(makeGoal(0.05), 0.20, 0.0, 0.6);
    EXPECT_NE(plan.mode, "REACHED");
    EXPECT_FALSE(controller.reached());
}

TEST(TerminalController, CaptureStopDoesNotLatchReached) {
    TerminalController controller;
    controller.setParams(makeParams());
    const auto capture = controller.updateAndPlan(makeGoal(0.50), 0.0, 0.0, 0.6);
    EXPECT_EQ(capture.mode, "TERMINAL_CAPTURE_STOP");
    EXPECT_FALSE(controller.reached());

    const auto still_capture = controller.updateAndPlan(makeGoal(0.50), 0.10, 0.20, 0.6);
    EXPECT_EQ(still_capture.mode, "TERMINAL_CAPTURE_STOP");
    EXPECT_FALSE(controller.reached());
}

TEST(TerminalController, NearGoalWithHighSpeedDoesNotLatchReached) {
    TerminalController controller;
    controller.setParams(makeParams());
    const auto fast = controller.updateAndPlan(makeGoal(0.05), 0.20, 0.0, 0.6);
    EXPECT_NE(fast.mode, "REACHED");
    EXPECT_FALSE(controller.reached());

    const auto outside_goal = controller.updateAndPlan(makeGoal(0.16), 0.0, 0.0, 0.6);
    EXPECT_NE(outside_goal.mode, "REACHED");
    EXPECT_FALSE(controller.reached());
}

TEST(TerminalController, ReachedUsesEuclideanDistanceNotRemainingS) {
    TerminalController controller;
    controller.setParams(makeParams());
    const auto goal = makeGoal(0.20, 1.0, 0.01);
    const auto plan = controller.updateAndPlan(goal, 0.0, 0.0, 0.6);
    EXPECT_NE(plan.mode, "REACHED");
    EXPECT_FALSE(controller.reached());
    EXPECT_DOUBLE_EQ(controller.diagnostics().remaining_s, 0.01);
    EXPECT_DOUBLE_EQ(controller.diagnostics().distance_to_goal, 0.20);
}

TEST(TerminalController, OmegaClampLimitsTerminalSpin) {
    TerminalController controller;
    controller.setParams(makeParams());
    const auto goal = makeGoal(0.50);
    const auto plan = controller.updateAndPlan(goal, 0.2, 0.0, 0.6);
    ASSERT_TRUE(plan.terminal_phase);
    const auto clamp = controller.clampCommand(0.2, 1.2, 0.2, 0.1, goal, plan, 0.6);
    EXPECT_NEAR(clamp.cmd_omega_post, 0.25, 1e-9);
}

TEST(TerminalController, NearGoalOmegaClampIsStricter) {
    TerminalController controller;
    controller.setParams(makeParams());
    const auto goal = makeGoal(0.20);
    const auto plan = controller.updateAndPlan(goal, 0.2, 0.0, 0.6);
    ASSERT_TRUE(plan.terminal_phase);
    const auto clamp = controller.clampCommand(0.2, -1.2, 0.2, 0.1, goal, plan, 0.6);
    EXPECT_NEAR(clamp.cmd_omega_post, -0.10, 1e-9);
}

TEST(TerminalController, GoalBehindForcesZeroDuringPendingStop) {
    TerminalController controller;
    controller.setParams(makeParams());
    const auto goal = makeGoal(0.4, -0.10);
    const auto plan = controller.updateAndPlan(goal, 0.2, 0.0, 0.6);
    const auto clamp = controller.clampCommand(0.2, 0.3, 0.2, 0.1, goal, plan, 0.6);
    EXPECT_DOUBLE_EQ(clamp.cmd_v_post, 0.0);
    EXPECT_DOUBLE_EQ(clamp.cmd_omega_post, 0.0);
}

TEST(TerminalController, RateLimitAvoidsInstantHardStop) {
    TerminalController controller;
    auto params = makeParams();
    params.capture_v_cap = 0.0;
    controller.setParams(params);
    const auto goal = makeGoal(0.4);
    const auto plan = controller.updateAndPlan(goal, 0.5, 0.0, 0.6);
    const auto clamp = controller.clampCommand(0.0, 0.0, 0.5, 0.1, goal, plan, 0.6);
    EXPECT_NEAR(clamp.cmd_v_post, 0.44, 1e-9);
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
