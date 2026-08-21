#include "spmpc_local_planner/runtime/execution_prediction/execution_state_predictor.h"
#include <gtest/gtest.h>
#include <cmath>

namespace spmpc_local_planner {
namespace {

StampNs stamp(double sec) {
    return secondsToNanoseconds(sec);
}

TimedCommandSample sample(double sec, double v, double omega) {
    TimedCommandSample out;
    out.stamp_ns = stamp(sec);
    out.command.linear = v;
    out.command.angular = omega;
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

TEST(ExecutionStatePredictor, SamplesLinearAndAngularDelayChannelsIndependently) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.80, 0.0, 1.0));
    history.push(sample(9.90, 1.0, 0.0));

    RobotState robot;
    SloshState slosh;
    auto p = params(0.20);
    p.linear_delay_sec = 0.10;
    p.angular_delay_sec = 0.20;
    const auto prediction = makePredictor().predict(
        robot, slosh, history, stamp(10.0), p);

    ASSERT_TRUE(prediction.valid);
    EXPECT_TRUE(prediction.history_complete);
    // Linear queries start at 9.90, while angular queries start at 9.80.
    // The angular channel turns for the first 100 ms, so the forward
    // displacement projected onto world x is slightly below 0.20 m.
    EXPECT_NEAR(prediction.predicted_robot.x, 0.20, 1e-3);
    EXPECT_GT(prediction.predicted_robot.y, 0.0);
    EXPECT_NEAR(prediction.predicted_robot.yaw, 0.10, 1e-6);
    EXPECT_NEAR(prediction.predicted_robot.v, 1.0, 1e-9);
    EXPECT_NEAR(prediction.predicted_robot.omega, 0.0, 1e-9);
}

TEST(ExecutionStatePredictor,
     FractionalCrossChannelDelaysPreservePublishedHistoryCausality) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.80, 0.0, 0.0));
    history.push(sample(9.90, 1.0, 2.0));

    auto p = params(0.20);
    p.linear_delay_sec = 0.05;
    p.angular_delay_sec = 0.13;
    p.max_integration_step_sec = 0.01;
    const auto prediction = makePredictor().predict(
        RobotState{}, SloshState{}, history, stamp(10.0), p);

    ASSERT_TRUE(prediction.valid);
    EXPECT_TRUE(prediction.history_complete);
    EXPECT_NEAR(prediction.integrated_duration_sec, 0.13, 1e-12);
    EXPECT_EQ(prediction.prediction_epoch_ns, 10130000000LL);
    // The fast linear channel sees the 9.90 command for the full common-front
    // rollout.  The slower angular channel sees it only after 30 ms.
    EXPECT_NEAR(prediction.predicted_robot.v, 1.0, 1e-12);
    EXPECT_NEAR(prediction.predicted_robot.omega, 2.0, 1e-12);
    EXPECT_NEAR(prediction.predicted_robot.yaw, 0.20, 1e-9);
    EXPECT_GT(prediction.predicted_robot.y, 0.0);
}

TEST(ExecutionStatePredictor, AppliesOptionalFirstOrderExecutionInertia) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.80, 1.0, 0.0));

    RobotState robot;
    SloshState slosh;
    auto p = params(0.20);
    p.linear_time_constant_sec = 0.20;
    const auto prediction = makePredictor().predict(
        robot, slosh, history, stamp(10.0), p);

    ASSERT_TRUE(prediction.valid);
    EXPECT_NEAR(prediction.predicted_robot.v, 1.0 - std::exp(-1.0), 1e-9);
    EXPECT_GT(prediction.predicted_robot.x, 0.0);
    EXPECT_LT(prediction.predicted_robot.x, 0.20);
}

TEST(ExecutionStatePredictor, ReportsFutureExecutionFrontEpoch) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.80, 1.0, 0.0));

    const auto prediction = makePredictor().predict(
        RobotState{}, SloshState{}, history, stamp(10.0), params(0.20));

    ASSERT_TRUE(prediction.valid);
    EXPECT_EQ(prediction.prediction_origin_epoch_ns, 10000000000LL);
    EXPECT_EQ(prediction.prediction_epoch_ns, 10200000000LL);
    EXPECT_NEAR(prediction.execution_lead_sec, 0.20, 1e-12);
    EXPECT_EQ(prediction.grid_execution_lead_steps, 10);
}

TEST(ExecutionStatePredictor, ExecutionTimingComesFromUnifiedModel) {
    const ExecutionStatePredictor predictor = makePredictor();
    auto p = params(0.20);
    p.linear_delay_sec = 0.15;
    p.angular_delay_sec = 0.22;
    double required_history_sec = 0.0;
    double execution_lead_sec = 0.0;
    int grid_execution_lead_steps = 0;

    ASSERT_TRUE(predictor.executionTiming(
        p, required_history_sec, execution_lead_sec,
        grid_execution_lead_steps));
    EXPECT_NEAR(required_history_sec, 0.22, 1e-12);
    EXPECT_NEAR(execution_lead_sec, 0.22, 1e-12);
    EXPECT_EQ(grid_execution_lead_steps, 11);
}

TEST(ExecutionStatePredictor, PropagatesStateAgeBeforeExecutionFront) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.70, 1.0, 0.0));

    const auto prediction = makePredictor().predict(
        RobotState{}, SloshState{}, history,
        stamp(9.90), stamp(10.0), params(0.20));

    ASSERT_TRUE(prediction.valid);
    EXPECT_TRUE(prediction.history_complete);
    EXPECT_NEAR(prediction.integrated_duration_sec, 0.30, 1e-12);
    EXPECT_NEAR(prediction.predicted_robot.x, 0.30, 1e-6);
    EXPECT_EQ(prediction.prediction_origin_epoch_ns, 9900000000LL);
    EXPECT_EQ(prediction.prediction_epoch_ns, 10200000000LL);
}

TEST(ExecutionStatePredictor, RejectsStateAgeBeyondPredictionContract) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    history.push(sample(9.40, 1.0, 0.0));
    auto p = params(0.20);
    p.max_prediction_sec = 0.40;

    const auto prediction = makePredictor().predict(
        RobotState{}, SloshState{}, history,
        stamp(9.50), stamp(10.0), p);

    EXPECT_FALSE(prediction.valid);
    EXPECT_EQ(prediction.status_code, DelayPhaseStatusCode::InvalidParams);
}

TEST(ExecutionStatePredictor, ZeroLeadReturnsIdentityWithoutHistory) {
    CommandHistoryBuffer history;
    history.configure(2.0);
    auto p = params(0.0);
    RobotState robot;
    robot.x = 1.2;
    robot.v = 0.3;

    const auto prediction = makePredictor().predict(
        robot, SloshState{}, history, stamp(10.0), p);

    ASSERT_TRUE(prediction.valid);
    EXPECT_TRUE(prediction.history_complete);
    EXPECT_DOUBLE_EQ(prediction.predicted_robot.x, robot.x);
    EXPECT_DOUBLE_EQ(prediction.predicted_robot.v, robot.v);
    EXPECT_EQ(prediction.prediction_epoch_ns, 10000000000LL);
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
    RobotState raw_robot;
    raw_robot.x = 1.0;
    raw_robot.y = 2.0;
    SloshState raw_slosh;
    raw_slosh.eta_x = 0.11;
    raw_slosh.eta_x_dot = -0.22;
    raw_slosh.eta_y = 0.33;
    raw_slosh.eta_y_dot = -0.44;

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

    const auto application = composeDelayPhaseState(
        raw_robot, raw_slosh, prediction,
        DelayPhaseMode::FixedRobotOnly, true);

    EXPECT_TRUE(application.robot_applied);
    EXPECT_FALSE(application.liquid_applied);
    EXPECT_TRUE(application.anyApplied());
    EXPECT_DOUBLE_EQ(application.robot.x, 3.0);
    EXPECT_DOUBLE_EQ(application.robot.y, 4.0);
    EXPECT_DOUBLE_EQ(application.slosh.eta_x, raw_slosh.eta_x);
    EXPECT_DOUBLE_EQ(application.slosh.eta_x_dot, raw_slosh.eta_x_dot);
    EXPECT_DOUBLE_EQ(application.slosh.eta_y, raw_slosh.eta_y);
    EXPECT_DOUBLE_EQ(application.slosh.eta_y_dot, raw_slosh.eta_y_dot);
}

TEST(DelayPhaseApplication, FixedClosedLoopStillReplacesRobotAndLiquid) {
    RobotState raw_robot;
    SloshState raw_slosh;
    raw_slosh.eta_x = 0.11;

    ExecutionStatePrediction prediction;
    prediction.valid = true;
    prediction.history_complete = true;
    prediction.status_code = DelayPhaseStatusCode::FixedClosedLoopOk;
    prediction.predicted_robot.x = 3.0;
    prediction.predicted_slosh.eta_x = 9.0;

    const auto application = composeDelayPhaseState(
        raw_robot, raw_slosh, prediction,
        DelayPhaseMode::FixedClosedLoop, true);

    EXPECT_TRUE(application.robot_applied);
    EXPECT_TRUE(application.liquid_applied);
    EXPECT_DOUBLE_EQ(application.robot.x, 3.0);
    EXPECT_DOUBLE_EQ(application.slosh.eta_x, 9.0);
}

TEST(DelayPhaseApplication, FailedExternalGuardAppliesNeitherState) {
    RobotState raw_robot;
    raw_robot.x = 1.0;
    SloshState raw_slosh;
    raw_slosh.eta_x = 0.11;

    ExecutionStatePrediction prediction;
    prediction.valid = true;
    prediction.history_complete = true;
    prediction.status_code = DelayPhaseStatusCode::FixedRobotOnlyOk;
    prediction.predicted_robot.x = 3.0;
    prediction.predicted_slosh.eta_x = 9.0;

    const auto application = composeDelayPhaseState(
        raw_robot, raw_slosh, prediction,
        DelayPhaseMode::FixedRobotOnly, false);

    EXPECT_FALSE(application.robot_applied);
    EXPECT_FALSE(application.liquid_applied);
    EXPECT_DOUBLE_EQ(application.robot.x, raw_robot.x);
    EXPECT_DOUBLE_EQ(application.slosh.eta_x, raw_slosh.eta_x);
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
