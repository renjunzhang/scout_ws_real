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
    return params;
}

TerminalGoalInfo makeGoal(double distance, double dx_robot = 1.0) {
    TerminalGoalInfo goal;
    goal.valid = true;
    goal.distance_to_goal = distance;
    goal.remaining_s = distance;
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
