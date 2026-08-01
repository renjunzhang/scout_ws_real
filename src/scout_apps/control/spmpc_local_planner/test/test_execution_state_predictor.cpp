#include "spmpc_local_planner/ros/execution_state_predictor.h"
#include <gtest/gtest.h>
#include <cmath>

namespace spmpc_local_planner {
namespace {

ros::Time stamp(double sec) {
    ros::Time t;
    t.fromSec(sec);
    return t;
}

TimedCommandSample sample(double sec, double v, double omega) {
    TimedCommandSample out;
    out.stamp = stamp(sec);
    out.cmd.linear.x = v;
    out.cmd.angular.z = omega;
    return out;
}

DelayPhaseParams params(double delay_sec = 0.20) {
    DelayPhaseParams p;
    p.mode = DelayPhaseMode::Shadow;
    p.linear_delay_sec = delay_sec;
    p.angular_delay_sec = delay_sec;
    p.max_prediction_sec = 0.40;
    p.max_integration_step_sec = 0.02;
    p.min_integration_step_sec = 0.001;
    p.cmd_timeout_sec = 0.50;
    p.require_complete_history = false;
    return p;
}

ExecutionStatePredictor makePredictor() {
    SloshModelParams slosh_params;
    slosh_params.dt = 0.02;
    ExecutionStatePredictor predictor;
    predictor.configure(slosh_params);
    return predictor;
}

}  // namespace

TEST(DelayPhaseTypes, ParsesFixedClosedLoopAliases) {
    EXPECT_EQ(parseDelayPhaseMode("fixed_closed_loop"), DelayPhaseMode::FixedClosedLoop);
    EXPECT_EQ(parseDelayPhaseMode("fixed-closed-loop"), DelayPhaseMode::FixedClosedLoop);
    EXPECT_EQ(parseDelayPhaseMode("p1_fixed_closed_loop"), DelayPhaseMode::FixedClosedLoop);
}

TEST(DelayPhaseTypes, ParsesFixedRobotOnlyAliases) {
    EXPECT_EQ(parseDelayPhaseMode("fixed_robot_only"), DelayPhaseMode::FixedRobotOnly);
    EXPECT_EQ(parseDelayPhaseMode("fixed-robot-only"), DelayPhaseMode::FixedRobotOnly);
    EXPECT_EQ(parseDelayPhaseMode("robot_only_closed_loop"), DelayPhaseMode::FixedRobotOnly);
    EXPECT_TRUE(isKnownDelayPhaseMode("FIXED_ROBOT_ONLY"));
    EXPECT_EQ(delayPhaseModeName(DelayPhaseMode::FixedRobotOnly), "fixed_robot_only");
}

TEST(ExecutionStatePredictor, ConstantStraightCommandPredictsForwardState) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.80, 1.0, 0.0));

    RobotState robot;
    SloshState slosh;
    const auto prediction = makePredictor().predict(robot, slosh, history, stamp(10.0), params(0.20));

    ASSERT_TRUE(prediction.valid);
    EXPECT_TRUE(prediction.history_complete);
    EXPECT_EQ(prediction.status_code, DelayPhaseStatusCode::ShadowOk);
    EXPECT_NEAR(prediction.predicted_robot.x, 0.20, 1e-6);
    EXPECT_NEAR(prediction.predicted_robot.y, 0.0, 1e-9);
    EXPECT_NEAR(prediction.predicted_robot.v, 1.0, 1e-9);
}

TEST(ExecutionStatePredictor, ConstantAngularCommandPredictsYaw) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.80, 0.0, 1.0));

    RobotState robot;
    SloshState slosh;
    const auto prediction = makePredictor().predict(robot, slosh, history, stamp(10.0), params(0.20));

    ASSERT_TRUE(prediction.valid);
    EXPECT_NEAR(prediction.predicted_robot.yaw, 0.20, 1e-6);
    EXPECT_NEAR(prediction.predicted_robot.omega, 1.0, 1e-9);
}

TEST(ExecutionStatePredictor, CompleteHistoryInFixedClosedLoopReportsClosedLoopOk) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.80, 1.0, 0.0));

    RobotState robot;
    SloshState slosh;
    auto p = params(0.20);
    p.mode = DelayPhaseMode::FixedClosedLoop;
    const auto prediction = makePredictor().predict(robot, slosh, history, stamp(10.0), p);

    ASSERT_TRUE(prediction.valid);
    EXPECT_TRUE(prediction.history_complete);
    EXPECT_EQ(prediction.status_code, DelayPhaseStatusCode::FixedClosedLoopOk);
    EXPECT_NEAR(prediction.predicted_robot.x, 0.20, 1e-6);
}

TEST(ExecutionStatePredictor, CompleteHistoryInFixedRobotOnlyReportsRobotOnlyOk) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.80, 1.0, 0.0));

    RobotState robot;
    SloshState slosh;
    auto p = params(0.20);
    p.mode = DelayPhaseMode::FixedRobotOnly;
    const auto prediction = makePredictor().predict(robot, slosh, history, stamp(10.0), p);

    ASSERT_TRUE(prediction.valid);
    EXPECT_TRUE(prediction.history_complete);
    EXPECT_EQ(prediction.status_code, DelayPhaseStatusCode::FixedRobotOnlyOk);
    EXPECT_EQ(prediction.status, "FIXED_ROBOT_ONLY_OK");
    EXPECT_NEAR(prediction.predicted_robot.x, 0.20, 1e-6);
}

TEST(DelayPhaseApplication, FixedRobotOnlyReplacesRobotButPreservesMeasuredLiquid) {
    SolverInput raw;
    raw.robot.x = 1.0;
    raw.robot.y = 2.0;
    raw.slosh.eta_x = 0.11;
    raw.slosh.eta_x_dot = -0.22;
    raw.slosh.eta_y = 0.33;
    raw.slosh.eta_y_dot = -0.44;

    ExecutionStatePrediction prediction;
    prediction.valid = true;
    prediction.history_complete = true;
    prediction.status_code = DelayPhaseStatusCode::FixedRobotOnlyOk;
    prediction.predicted_robot.x = 3.0;
    prediction.predicted_robot.y = 4.0;
    prediction.predicted_slosh.eta_x = 9.0;
    prediction.predicted_slosh.eta_x_dot = 8.0;
    prediction.predicted_slosh.eta_y = 7.0;
    prediction.predicted_slosh.eta_y_dot = 6.0;

    const auto application = composeDelayPhaseSolverInput(
        raw, prediction, DelayPhaseMode::FixedRobotOnly, true);

    EXPECT_TRUE(application.robot_applied);
    EXPECT_FALSE(application.liquid_applied);
    EXPECT_TRUE(application.anyApplied());
    EXPECT_DOUBLE_EQ(application.solver_input.robot.x, 3.0);
    EXPECT_DOUBLE_EQ(application.solver_input.robot.y, 4.0);
    EXPECT_DOUBLE_EQ(application.solver_input.slosh.eta_x, raw.slosh.eta_x);
    EXPECT_DOUBLE_EQ(application.solver_input.slosh.eta_x_dot, raw.slosh.eta_x_dot);
    EXPECT_DOUBLE_EQ(application.solver_input.slosh.eta_y, raw.slosh.eta_y);
    EXPECT_DOUBLE_EQ(application.solver_input.slosh.eta_y_dot, raw.slosh.eta_y_dot);
}

TEST(DelayPhaseApplication, FixedClosedLoopStillReplacesRobotAndLiquid) {
    SolverInput raw;
    raw.slosh.eta_x = 0.11;

    ExecutionStatePrediction prediction;
    prediction.valid = true;
    prediction.history_complete = true;
    prediction.status_code = DelayPhaseStatusCode::FixedClosedLoopOk;
    prediction.predicted_robot.x = 3.0;
    prediction.predicted_slosh.eta_x = 9.0;

    const auto application = composeDelayPhaseSolverInput(
        raw, prediction, DelayPhaseMode::FixedClosedLoop, true);

    EXPECT_TRUE(application.robot_applied);
    EXPECT_TRUE(application.liquid_applied);
    EXPECT_DOUBLE_EQ(application.solver_input.robot.x, 3.0);
    EXPECT_DOUBLE_EQ(application.solver_input.slosh.eta_x, 9.0);
}

TEST(DelayPhaseApplication, FailedExternalGuardAppliesNeitherState) {
    SolverInput raw;
    raw.robot.x = 1.0;
    raw.slosh.eta_x = 0.11;

    ExecutionStatePrediction prediction;
    prediction.valid = true;
    prediction.history_complete = true;
    prediction.status_code = DelayPhaseStatusCode::FixedRobotOnlyOk;
    prediction.predicted_robot.x = 3.0;
    prediction.predicted_slosh.eta_x = 9.0;

    const auto application = composeDelayPhaseSolverInput(
        raw, prediction, DelayPhaseMode::FixedRobotOnly, false);

    EXPECT_FALSE(application.robot_applied);
    EXPECT_FALSE(application.liquid_applied);
    EXPECT_DOUBLE_EQ(application.solver_input.robot.x, raw.robot.x);
    EXPECT_DOUBLE_EQ(application.solver_input.slosh.eta_x, raw.slosh.eta_x);
}

TEST(ExecutionStatePredictor, PartialHistoryIsReportedButAllowedByDefault) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.90, 1.0, 0.0));

    RobotState robot;
    SloshState slosh;
    auto p = params(0.40);
    const auto prediction = makePredictor().predict(robot, slosh, history, stamp(10.0), p);

    ASSERT_TRUE(prediction.valid);
    EXPECT_FALSE(prediction.history_complete);
    EXPECT_EQ(prediction.status_code, DelayPhaseStatusCode::PartialHistory);
    EXPECT_NEAR(prediction.missing_history_sec, 0.30, 1e-9);
    EXPECT_NEAR(prediction.predicted_robot.x, 0.10, 1e-6);
}

TEST(ExecutionStatePredictor, RequireCompleteHistoryInvalidatesPartialPrediction) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.90, 1.0, 0.0));

    RobotState robot;
    SloshState slosh;
    auto p = params(0.40);
    p.require_complete_history = true;
    const auto prediction = makePredictor().predict(robot, slosh, history, stamp(10.0), p);

    EXPECT_FALSE(prediction.valid);
    EXPECT_EQ(prediction.status_code, DelayPhaseStatusCode::PartialHistory);
}

TEST(ExecutionStatePredictor, EmptyHistoryIsInvalid) {
    CommandHistoryBuffer history;
    history.configure(2.0);

    RobotState robot;
    SloshState slosh;
    const auto prediction = makePredictor().predict(robot, slosh, history, stamp(10.0), params(0.20));

    EXPECT_FALSE(prediction.valid);
    EXPECT_EQ(prediction.status_code, DelayPhaseStatusCode::NoCmdHistory);
}

TEST(ExecutionStatePredictor, DoesNotMutateInputSloshState) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.80, 1.0, 0.2));

    RobotState robot;
    SloshState slosh;
    slosh.eta_x = 0.10;
    slosh.eta_y = -0.05;
    const auto prediction = makePredictor().predict(robot, slosh, history, stamp(10.0), params(0.20));

    ASSERT_TRUE(prediction.valid);
    EXPECT_DOUBLE_EQ(slosh.eta_x, 0.10);
    EXPECT_DOUBLE_EQ(slosh.eta_y, -0.05);
    EXPECT_DOUBLE_EQ(prediction.raw_slosh.eta_x, 0.10);
    EXPECT_DOUBLE_EQ(prediction.raw_slosh.eta_y, -0.05);
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
