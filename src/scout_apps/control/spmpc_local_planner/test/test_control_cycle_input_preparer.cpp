#include "spmpc_local_planner/controller/control_cycle_input_preparer.h"

#include <gtest/gtest.h>

namespace spmpc_local_planner {
namespace {

constexpr StampNs kSelectionNs = 10000000000LL;
constexpr StampNs kStateNs = 9990000000LL;

SloshObserverHealth healthyOdom(StampNs state_stamp_ns = kStateNs) {
    SloshObserverHealth health;
    health.snapshot.configured = true;
    health.snapshot.valid = true;
    health.snapshot.update_count = 10;
    health.snapshot.state_stamp_ns = state_stamp_ns;
    health.snapshot.state.eta_x = 0.01;
    health.snapshot.state.eta_x_dot = 0.02;
    health.snapshot.state.eta_y = 0.03;
    health.snapshot.state.eta_y_dot = 0.04;
    health.input_ready = true;
    return health;
}

RobotState markedRobot() {
    RobotState state;
    state.x = 1.0;
    state.y = 2.0;
    state.yaw = 0.1;
    state.v = 0.2;
    state.omega = 0.05;
    return state;
}

RobotStateLookup successfulLookup(StampNs* observed_target = nullptr) {
    return [observed_target](StampNs target_epoch_ns) {
        if (observed_target) {
            *observed_target = target_epoch_ns;
        }
        RobotStateLookupResult result;
        result.valid = true;
        result.state = markedRobot();
        result.interpolated = target_epoch_ns > 0;
        result.status = target_epoch_ns > 0 ? "INTERPOLATED" : "LATEST";
        return result;
    };
}

ControlCycleInputRequest baseRequest() {
    ControlCycleInputRequest request;
    request.cycle_id = 7;
    request.cycle_start_ns = kSelectionNs - 20000000LL;
    request.selection_time_ns = kSelectionNs;
    request.prediction_evaluation_ns = kSelectionNs;
    request.raw_robot_state_stamp_ns = kStateNs;
    request.last_odom_receive_ns = kStateNs;
    request.odom_observer = healthyOdom();
    request.solver_consumes_selected_state = true;
    request.state_timing.require_common_epoch = true;
    request.state_timing.max_raw_skew_sec = 0.08;
    request.dt = 0.02;
    request.horizon_steps = 20;
    request.delay_phase.mode = DelayPhaseMode::Off;
    request.robot_state_lookup = successfulLookup();
    return request;
}

ControlCycleInputPreparer configuredPreparer() {
    ControlCycleInputPreparer preparer;
    SloshObserverSelectorParams observer_params;
    observer_params.nominal_source = SloshObserverSource::Odom;
    EXPECT_TRUE(preparer.configureObserver(observer_params));
    SloshModelParams slosh_params;
    slosh_params.dt = 0.02;
    EXPECT_TRUE(preparer.configurePrediction(slosh_params));
    return preparer;
}

CommandHistoryBuffer completeHistory() {
    CommandHistoryBuffer history;
    history.configure(2.0);
    TimedCommandSample old_sample;
    old_sample.stamp_ns = 9700000000LL;
    old_sample.command.linear = 0.5;
    old_sample.command.angular = 0.1;
    history.push(old_sample);
    TimedCommandSample current_sample = old_sample;
    current_sample.stamp_ns = kSelectionNs;
    history.push(current_sample);
    return history;
}

ExecutionModelContract formalExecutionContract() {
    ExecutionModelContract contract;
    contract.contract_id = "formal_preparer_test_v1";
    contract.contract_hash = "formal-preparer-test-hash";
    contract.dt = 0.02;
    contract.linear.delay_sec = 0.03;
    contract.angular.delay_sec = 0.05;
    contract.linear.output_min = -1.0;
    contract.linear.output_max = 1.0;
    contract.angular.output_min = -1.0;
    contract.angular.output_max = 1.0;
    return contract;
}

CommandHistoryBuffer formalHistory() {
    CommandHistoryBuffer history;
    history.configure(2.0);
    for (int index = 0; index < 5; ++index) {
        TimedCommandSample sample;
        sample.stamp_ns = 9900000000LL +
            static_cast<StampNs>(index) * 20000000LL;
        sample.command.linear = 0.1 * static_cast<double>(index + 1);
        sample.command.angular = 0.05 * static_cast<double>(index + 1);
        history.push(sample);
    }
    return history;
}

void configureFormalHorizon(ControlCycleInputPreparer& preparer) {
    ExecutionHorizonBuilderConfig config;
    config.command_timeout_sec = 0.2;
    config.max_alignment_sec = 0.2;
    config.max_integration_step_sec = 0.01;
    config.min_integration_step_sec = 0.001;
    std::string error;
    EXPECT_TRUE(preparer.configureExecutionHorizon(
        formalExecutionContract(), config, error)) << error;
}

PublishEpochEstimate publishEstimate(
    const ControlCycleInputRequest& request,
    bool enabled,
    double estimated_dc_sec) {
    PublishLatencyModel model;
    PublishLatencyModelConfig config;
    config.enabled = enabled;
    config.estimated_dc_sec = estimated_dc_sec;
    std::string error;
    EXPECT_TRUE(model.configure(config, error)) << error;
    CycleTimingContract cycle;
    cycle.cycle_id = request.cycle_id;
    cycle.cycle_start_stamp_ns = request.cycle_start_ns;
    cycle.control_period_sec = request.dt;
    return model.estimate(cycle);
}

void enableClosedLoop(ControlCycleInputRequest& request,
                      CommandHistoryBuffer& history,
                      DelayPhaseMode mode = DelayPhaseMode::FixedClosedLoop) {
    request.delay_phase.mode = mode;
    request.delay_phase.linear_delay_sec = 0.20;
    request.delay_phase.angular_delay_sec = 0.20;
    request.delay_phase.max_prediction_sec = 0.40;
    request.delay_phase.max_integration_step_sec = 0.02;
    request.delay_phase.min_integration_step_sec = 0.001;
    request.delay_phase.cmd_timeout_sec = 0.50;
    request.delay_phase.odom_timeout_sec = 0.50;
    request.command_history = &history;
}

}  // namespace

TEST(ControlCycleInputPreparer, ObserverFailureIsFailClosedBeforeRobotLookup) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    request.odom_observer.snapshot.valid = false;
    request.odom_observer.input_ready = false;
    bool lookup_called = false;
    request.robot_state_lookup = [&lookup_called](StampNs) {
        lookup_called = true;
        return RobotStateLookupResult{};
    };

    const auto result = preparer.prepare(request);

    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.failure, ControlInputFailure::ObserverUnavailable);
    EXPECT_FALSE(lookup_called);
    EXPECT_NE(result.status.find("WAITING_FOR_SLOSH_OBSERVER_"),
              std::string::npos);
}

TEST(ControlCycleInputPreparer, RejectsRawStateSkewBeforeCommonEpochLookup) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    request.odom_observer = healthyOdom(9900000000LL);
    request.raw_robot_state_stamp_ns = kSelectionNs;
    bool lookup_called = false;
    request.robot_state_lookup = [&lookup_called](StampNs) {
        lookup_called = true;
        return RobotStateLookupResult{};
    };

    const auto result = preparer.prepareState(request);

    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.failure, ControlInputFailure::RawStateSkew);
    EXPECT_FALSE(lookup_called);
    EXPECT_NEAR(result.timing.raw_state_skew_sec, 0.10, 1e-12);
    EXPECT_EQ(result.timing.state_alignment_status,
              "RAW_STATE_SKEW_CONTRACT_FAILED");
}

TEST(ControlCycleInputPreparer, CommonEpochLookupPopulatesAlignedTiming) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    StampNs observed_target = 0;
    request.robot_state_lookup = successfulLookup(&observed_target);

    const auto result = preparer.prepareState(request);

    ASSERT_TRUE(result.ready);
    EXPECT_EQ(observed_target, kStateNs);
    EXPECT_DOUBLE_EQ(result.raw_input.robot.x, 1.0);
    EXPECT_EQ(result.timing.robot_state_stamp_ns, kStateNs);
    EXPECT_EQ(result.timing.liquid_state_stamp_ns, kStateNs);
    EXPECT_EQ(result.timing.solver_input_epoch_ns, kStateNs);
    EXPECT_TRUE(result.timing.state_time_aligned);
    EXPECT_TRUE(result.timing.robot_state_interpolated);
    EXPECT_EQ(result.timing.state_alignment_status, "INTERPOLATED");
}

TEST(ControlCycleInputPreparer, CommonEpochLookupFailurePreservesReason) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    request.robot_state_lookup = [](StampNs) {
        RobotStateLookupResult result;
        result.status = "TF_AT_COMMON_EPOCH_UNAVAILABLE";
        return result;
    };

    const auto result = preparer.prepareState(request);

    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.failure,
              ControlInputFailure::CommonEpochRobotUnavailable);
    EXPECT_EQ(result.timing.state_alignment_status,
              "TF_AT_COMMON_EPOCH_UNAVAILABLE");
    EXPECT_EQ(result.status,
              "STATE_TIME_ALIGNMENT_FAILED_TF_AT_COMMON_EPOCH_UNAVAILABLE");
}

TEST(ControlCycleInputPreparer, LatestPoseFailureRequestsEarlyTfStatus) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    request.solver_consumes_selected_state = false;
    request.robot_state_lookup = [](StampNs target_epoch_ns) {
        EXPECT_EQ(target_epoch_ns, 0);
        return RobotStateLookupResult{};
    };

    const auto result = preparer.prepareState(request);

    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.failure, ControlInputFailure::LatestRobotUnavailable);
    EXPECT_TRUE(result.publish_early_delay_status);
    EXPECT_EQ(result.delay_phase_status, DelayPhaseStatusCode::NoTfPose);
    EXPECT_EQ(result.status, "WAITING_FOR_TF_POSE");
}

TEST(ControlCycleInputPreparer, PredictionOffKeepsIdentitySolverInput) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();

    const auto result = preparer.prepare(request);

    ASSERT_TRUE(result.ready);
    EXPECT_FALSE(result.have_prediction);
    EXPECT_EQ(result.prediction.status_code, DelayPhaseStatusCode::Off);
    EXPECT_DOUBLE_EQ(result.prediction.predicted_robot.x,
                     result.raw_input.robot.x);
    EXPECT_DOUBLE_EQ(result.solver_input.robot.x,
                     result.raw_input.robot.x);
    EXPECT_FALSE(result.robot_delay_compensation_applied);
    EXPECT_FALSE(result.liquid_delay_compensation_applied);
    EXPECT_FALSE(result.execution_horizon_active);
    EXPECT_FALSE(result.solver_input.execution_horizon.active);
}

TEST(ControlCycleInputPreparer,
     ExpectedPublishEpochDrivesPredictionAndSolverInput) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    auto history = completeHistory();
    enableClosedLoop(request, history);
    request.publish_epoch_estimate = publishEstimate(
        request, true, 0.04);

    const auto result = preparer.prepare(request);

    ASSERT_TRUE(result.ready);
    ASSERT_TRUE(result.prediction.valid);
    EXPECT_TRUE(result.prediction_uses_expected_publish_epoch);
    EXPECT_EQ(result.prediction_evaluation_epoch_ns, 10020000000LL);
    EXPECT_NEAR(result.prediction.integrated_duration_sec, 0.23, 1e-12);
    EXPECT_EQ(result.prediction.prediction_epoch_ns, 10220000000LL);
    EXPECT_EQ(result.execution_front_steps, 10);
    EXPECT_EQ(
        result.solver_input.publish_epoch_estimate
            .expected_publish_stamp_ns,
        10020000000LL);
    EXPECT_EQ(result.solver_input.cycle_timing.expected_publish_stamp_ns,
              10020000000LL);
    EXPECT_TRUE(
        result.solver_input.cycle_timing.publish_epoch_estimate_valid);
}

TEST(ControlCycleInputPreparer,
     EstimateOffPreservesExplicitPredictionEvaluationEpoch) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    auto history = completeHistory();
    enableClosedLoop(request, history);
    request.publish_epoch_estimate = publishEstimate(
        request, false, 0.0);

    const auto result = preparer.prepare(request);

    ASSERT_TRUE(result.ready);
    ASSERT_TRUE(result.prediction.valid);
    EXPECT_FALSE(result.prediction_uses_expected_publish_epoch);
    EXPECT_EQ(result.prediction_evaluation_epoch_ns, kSelectionNs);
    EXPECT_EQ(result.prediction.prediction_epoch_ns, 10200000000LL);
    EXPECT_EQ(result.solver_input.publish_epoch_estimate.status,
              "ESTIMATE_OFF");
    EXPECT_EQ(result.solver_input.cycle_timing.publish_deadline_stamp_ns,
              kSelectionNs);
}

TEST(ControlCycleInputPreparer,
     RejectsMismatchedPublishEpochBeforeStateLookup) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    request.publish_epoch_estimate = publishEstimate(
        request, true, 0.01);
    request.publish_epoch_estimate.expected_publish_stamp_ns += 1;
    bool lookup_called = false;
    request.robot_state_lookup = [&lookup_called](StampNs) {
        lookup_called = true;
        return RobotStateLookupResult{};
    };

    const auto result = preparer.prepareState(request);

    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.failure, ControlInputFailure::PublishEpochContract);
    EXPECT_EQ(result.status, "PUBLISH_EPOCH_CONTRACT_MISMATCH");
    EXPECT_FALSE(lookup_called);
}

TEST(ControlCycleInputPreparer, StaleOdomSuppressesClosedLoopApplication) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    auto history = completeHistory();
    enableClosedLoop(request, history);
    request.last_odom_receive_ns = 9000000000LL;

    const auto result = preparer.prepare(request);

    ASSERT_TRUE(result.ready);
    ASSERT_TRUE(result.prediction.valid);
    EXPECT_EQ(result.delay_phase_status, DelayPhaseStatusCode::OdomStale);
    EXPECT_FALSE(result.robot_delay_compensation_applied);
    EXPECT_FALSE(result.liquid_delay_compensation_applied);
    EXPECT_DOUBLE_EQ(result.solver_input.robot.x,
                     result.raw_input.robot.x);
}

TEST(ControlCycleInputPreparer, FixedClosedLoopAppliesBothStatesAndTiming) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    auto history = completeHistory();
    enableClosedLoop(request, history);

    const auto result = preparer.prepare(request);

    ASSERT_TRUE(result.ready);
    ASSERT_TRUE(result.prediction.valid);
    EXPECT_TRUE(result.robot_delay_compensation_applied);
    EXPECT_TRUE(result.liquid_delay_compensation_applied);
    EXPECT_TRUE(result.solver_origin_at_execution_front);
    EXPECT_EQ(result.execution_front_steps, 10);
    EXPECT_GT(result.solver_input.robot.x, result.raw_input.robot.x);
    EXPECT_DOUBLE_EQ(result.solver_input.dt, result.raw_input.dt);
    EXPECT_EQ(result.solver_input.horizon_steps,
              result.raw_input.horizon_steps);
    EXPECT_EQ(result.timing.solver_input_epoch_ns, 10200000000LL);
    EXPECT_EQ(result.timing.robot_state_stamp_ns, 10200000000LL);
    EXPECT_EQ(result.timing.liquid_state_stamp_ns, 10200000000LL);
    EXPECT_TRUE(result.timing.state_time_aligned);
    EXPECT_EQ(result.timing.state_alignment_status,
              "DELAY_PREDICTED_COMMON_EPOCH");
}

TEST(ControlCycleInputPreparer, RejectsPartialClosedLoopStateApplication) {
    auto preparer = configuredPreparer();
    auto request = baseRequest();
    auto history = completeHistory();
    enableClosedLoop(request, history, DelayPhaseMode::FixedRobotOnly);

    const auto result = preparer.prepare(request);

    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.failure,
              ControlInputFailure::PartialDelayStateApplication);
    EXPECT_TRUE(result.robot_delay_compensation_applied);
    EXPECT_FALSE(result.liquid_delay_compensation_applied);
    EXPECT_FALSE(result.timing.state_time_aligned);
    EXPECT_EQ(result.timing.state_alignment_status,
              "PARTIAL_DELAY_STATE_APPLICATION_FORBIDDEN");
    EXPECT_EQ(result.status,
              "STATE_TIME_ALIGNMENT_FAILED_DELAY_PHASE");
}

TEST(ControlCycleInputPreparer,
     ExplicitFormalRequestBuildsSolverExecutionHorizon) {
    auto preparer = configuredPreparer();
    configureFormalHorizon(preparer);
    auto request = baseRequest();
    auto history = formalHistory();
    request.publish_epoch_estimate = publishEstimate(
        request, true, 0.015);
    request.command_history = &history;
    request.execution_horizon_requested = true;
    request.execution_contract_hash =
        formalExecutionContract().contract_hash;
    request.execution_initial_progress_s = 1.2;
    request.execution_liquid_horizon_steps = 2;

    const auto result = preparer.prepare(request);

    ASSERT_TRUE(result.ready) << result.status;
    ASSERT_TRUE(result.execution_horizon_build.valid)
        << result.execution_horizon_build.status;
    EXPECT_TRUE(result.execution_horizon_active);
    ASSERT_TRUE(result.solver_input.execution_horizon.active);
    EXPECT_FALSE(result.have_prediction);
    EXPECT_EQ(result.solver_input.execution_horizon.initial_epoch_ns,
              9995000000LL);
    EXPECT_EQ(result.solver_input.execution_horizon.execution_front_steps,
              3);
    EXPECT_EQ(result.solver_input.execution_horizon.horizon_steps, 5);
    EXPECT_EQ(
        result.solver_input.execution_horizon.physical_front_epoch_ns,
        10045000000LL);
    EXPECT_EQ(result.solver_input.execution_horizon.grid_front_epoch_ns,
              10055000000LL);
    EXPECT_EQ(result.solver_input.execution_horizon.terminal_epoch_ns,
              10095000000LL);
    ASSERT_EQ(
        result.solver_input.execution_horizon.initial_state.linear
            .pending_commands.size(),
        2u);
    ASSERT_EQ(
        result.solver_input.execution_horizon.initial_state.angular
            .pending_commands.size(),
        3u);
}

TEST(ControlCycleInputPreparer,
     FormalRequestFailsClosedOnContractHashMutation) {
    auto preparer = configuredPreparer();
    configureFormalHorizon(preparer);
    auto request = baseRequest();
    auto history = formalHistory();
    request.publish_epoch_estimate = publishEstimate(
        request, true, 0.015);
    request.command_history = &history;
    request.execution_horizon_requested = true;
    request.execution_contract_hash = "mutated-hash";
    request.execution_liquid_horizon_steps = 2;

    const auto result = preparer.prepare(request);

    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.failure,
              ControlInputFailure::ExecutionHorizonContext);
    EXPECT_EQ(result.status, "EXECUTION_CONTRACT_HASH_MISMATCH");
    EXPECT_FALSE(result.solver_input.execution_horizon.active);
}

TEST(ControlCycleInputPreparer,
     FormalRequestFailsClosedWhenPublishEstimateIsOff) {
    auto preparer = configuredPreparer();
    configureFormalHorizon(preparer);
    auto request = baseRequest();
    auto history = formalHistory();
    request.publish_epoch_estimate = publishEstimate(
        request, false, 0.0);
    request.command_history = &history;
    request.execution_horizon_requested = true;
    request.execution_contract_hash =
        formalExecutionContract().contract_hash;
    request.execution_liquid_horizon_steps = 2;

    const auto result = preparer.prepare(request);

    EXPECT_FALSE(result.ready);
    EXPECT_EQ(result.failure,
              ControlInputFailure::ExecutionHorizonContext);
    EXPECT_EQ(result.status, "INVALID_PUBLISH_EPOCH_ESTIMATE");
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
