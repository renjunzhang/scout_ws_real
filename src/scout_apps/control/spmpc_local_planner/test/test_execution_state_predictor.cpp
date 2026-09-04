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
    predictor.configure(slosh_params, 0.01);
    return predictor;
}

ActuatorModelParams actuatorParams() {
    ActuatorModelParams p;
    p.mode = ExecutionModelMode::ExplicitActuator;
    p.dt = 1.0 / 30.0;
    p.linear_delay_sec = kExplicitLinearDelaySteps * p.dt;
    p.angular_delay_sec = kExplicitAngularDelaySteps * p.dt;
    p.linear_tau_sec = 0.112;
    p.angular_tau_sec = 0.119;
    p.linear_gain = 1.018;
    p.angular_gain = 1.096;
    p.max_prefix_prediction_sec = 0.20;
    p.max_integration_step_sec = 0.01;
    p.require_complete_history = true;
    return p;
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

TEST(ExecutionStatePredictor, ExplicitActuatorBuildsIndependentFifos) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    const double dt = 1.0 / 30.0;
    for (int i = 0; i <= 36; ++i) {
        history.push(sample(8.8 + i * dt, 0.01 * i, -0.02 * i));
    }

    RobotState robot;
    SloshState slosh;
    const auto prediction = makePredictor().predictExplicitActuator(
        robot, slosh, history, stamp(10.0), stamp(10.0), actuatorParams());

    ASSERT_TRUE(prediction.valid) << prediction.status;
    ASSERT_TRUE(prediction.actuator.valid);
    EXPECT_TRUE(prediction.history_complete);
    EXPECT_NEAR(prediction.actuator.v_cmd, 0.36, 1.0e-9);
    EXPECT_NEAR(prediction.actuator.omega_cmd, -0.72, 1.0e-9);
    EXPECT_NEAR(prediction.actuator.a_cmd_memory, 0.30, 1.0e-9);
    for (int i = 0; i < kExplicitLinearDelaySteps; ++i) {
        EXPECT_NEAR(
            prediction.actuator.linear_delay_queue[static_cast<size_t>(i)],
            0.01 * (31 + i),
            1.0e-9);
    }
    for (int i = 0; i < kExplicitAngularDelaySteps; ++i) {
        EXPECT_NEAR(
            prediction.actuator.angular_delay_queue[static_cast<size_t>(i)],
            -0.02 * (26 + i),
            1.0e-9);
    }
}

TEST(ExecutionStatePredictor, ExplicitActuatorAccelerationMemoryUsesEmittedCommands) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    const double dt = 1.0 / 30.0;
    for (int i = 0; i <= 12; ++i) {
        const double v = i < 12 ? 0.20 : 0.17;
        history.push(sample(10.0 - (12 - i) * dt, v, 0.0));
    }

    RobotState robot;
    SloshState slosh;
    const auto prediction = makePredictor().predictExplicitActuator(
        robot, slosh, history, stamp(10.0), stamp(10.0), actuatorParams());

    ASSERT_TRUE(prediction.valid) << prediction.status;
    EXPECT_NEAR(prediction.actuator.v_cmd, 0.17, 1.0e-12);
    EXPECT_NEAR(prediction.actuator.linear_delay_queue.back(), 0.20, 1.0e-12);
    EXPECT_NEAR(prediction.actuator.a_cmd_memory, -0.90, 1.0e-9);
}

TEST(ExecutionStatePredictor, ExplicitActuatorPropagatesKnownPrefixWithFopdt) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.0, 0.20, 0.30));
    history.push(sample(10.0, 0.20, 0.30));

    RobotState robot;
    SloshState slosh;
    const auto p = actuatorParams();
    const auto prediction = makePredictor().predictExplicitActuator(
        robot, slosh, history, stamp(9.9), stamp(10.0), p);

    ASSERT_TRUE(prediction.valid) << prediction.status;
    const double expected_v =
        p.linear_gain * 0.20 * (1.0 - std::exp(-0.10 / p.linear_tau_sec));
    const double expected_omega =
        p.angular_gain * 0.30 *
        (1.0 - std::exp(-0.10 / p.angular_tau_sec));
    EXPECT_NEAR(prediction.predicted_robot.v, expected_v, 1.0e-9);
    EXPECT_NEAR(prediction.predicted_robot.omega, expected_omega, 1.0e-9);
    EXPECT_NEAR(
        prediction.actuator.a_actual,
        (p.linear_gain * 0.20 - expected_v) / p.linear_tau_sec,
        1.0e-9);
    EXPECT_NEAR(
        prediction.actuator.alpha_actual,
        (p.angular_gain * 0.30 - expected_omega) / p.angular_tau_sec,
        1.0e-9);
}

TEST(ExecutionStatePredictor, ExplicitActuatorPropagatesVeryShortLiquidTail) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.0, 0.20, 0.0));
    history.push(sample(10.0, 0.20, 0.0));

    RobotState robot;
    SloshState slosh;
    slosh.eta_x = 0.001;
    slosh.eta_x_dot = 0.10;
    const auto prediction = makePredictor().predictExplicitActuator(
        robot,
        slosh,
        history,
        stamp(9.99995),
        stamp(10.0),
        actuatorParams());

    ASSERT_TRUE(prediction.valid) << prediction.status;
    EXPECT_TRUE(std::isfinite(prediction.predicted_slosh.eta_x));
    EXPECT_GT(
        std::abs(prediction.predicted_slosh.eta_x - slosh.eta_x),
        1.0e-9);
}

TEST(ExecutionStatePredictor, ExplicitActuatorFailsClosedOnIncompleteFifo) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.8, 0.20, 0.30));

    RobotState robot;
    SloshState slosh;
    const auto prediction = makePredictor().predictExplicitActuator(
        robot, slosh, history, stamp(10.0), stamp(10.0), actuatorParams());

    EXPECT_FALSE(prediction.valid);
    EXPECT_FALSE(prediction.actuator.valid);
    EXPECT_EQ(prediction.status, "INCOMPLETE_ANGULAR_DELAY_QUEUE");
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
