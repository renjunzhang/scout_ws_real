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

TEST(TerminalController, HandoffDoesNotStopInSlowdownOrCaptureZone) {
    auto params = makeParams();
    params.mpc_stop_handoff_enable = true;
    TerminalController controller;
    controller.setParams(params);
    EXPECT_FALSE(controller.updateAndPlan(makeGoal(1.0), 0.2, 0.0, 0.6).owns_command);
    EXPECT_FALSE(controller.updateAndPlan(makeGoal(0.5), 0.0, 0.0, 0.6).owns_command);
    EXPECT_FALSE(controller.reached());
}

TEST(TerminalController, HandoffPersistsUntilNewTaskReset) {
    auto params = makeParams();
    params.mpc_stop_handoff_enable = true;
    TerminalController controller;
    controller.setParams(params);
    const auto stop = controller.updateAndPlan(makeGoal(0.1), 0.1, 0.1, 0.6);
    EXPECT_TRUE(stop.owns_command);
    EXPECT_EQ(stop.mode, "TERMINAL_STOP");
    EXPECT_FALSE(controller.reached());
    EXPECT_TRUE(controller.diagnostics().command_owned);
    EXPECT_TRUE(controller.updateAndPlan(makeGoal(0.9), 0.1, 0.1, 0.6).owns_command);
    controller.clearPending();
    EXPECT_TRUE(controller.updateAndPlan(makeGoal(0.9), 0.1, 0.1, 0.6).owns_command);
    controller.reset();
    EXPECT_FALSE(controller.updateAndPlan(makeGoal(0.9), 0.1, 0.1, 0.6).owns_command);
}

TEST(TerminalController, StopUsesPublishedCommandAndNeverReacceleratesFromZero) {
    TerminalController controller;
    const auto stop = controller.stopCommand(0.025937068134, -0.046557832894,
                                            1.0 / 30.0, 0.6, 1.2);
    EXPECT_NEAR(stop.cmd_v_post, 0.005937068134, 1e-10);
    EXPECT_NEAR(stop.cmd_omega_post, -0.006557832894, 1e-10);
    const auto zero = controller.stopCommand(0.0, 0.0, 1.0 / 30.0, 0.6, 1.2);
    EXPECT_DOUBLE_EQ(zero.cmd_v_post, 0.0);
    EXPECT_DOUBLE_EQ(zero.cmd_omega_post, 0.0);
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
