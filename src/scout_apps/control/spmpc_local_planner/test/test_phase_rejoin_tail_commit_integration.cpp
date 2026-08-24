#include "spmpc_local_planner/phase_rejoin/bounded_tracking_recovery_policy.h"
#include "spmpc_local_planner/controller/control_cycle_engine.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"

#include "../generated/acados/spmpc_delay_augmented_phase_solver_manifest.h"

#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <map>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

constexpr double kPathLength = 0.09;
constexpr std::size_t kArtifactSize = 20;
constexpr std::size_t kMaxTailIndex = kArtifactSize -
    static_cast<std::size_t>(manifest::kHorizonSteps) - 1u;

std::string number(double value) {
    std::ostringstream out;
    out << std::setprecision(17) << value;
    return out.str();
}

SloshModelParams manifestSlosh() {
    SloshModelParams slosh;
    slosh.container_radius = manifest::kContainerRadius;
    slosh.liquid_height = manifest::kLiquidHeight;
    slosh.liquid_density = manifest::kLiquidDensity;
    slosh.damping_ratio = manifest::kDampingRatio;
    slosh.mode_index = manifest::kModeIndex;
    slosh.dt = manifest::kDt;
    slosh.slosh_height_ref = manifest::kSloshHeightRef;
    slosh.slosh_eta_dot_ratio = manifest::kSloshEtaDotRatio;
    slosh.use_linear_model = manifest::kUseLinearModel;
    slosh.use_parabola_term = manifest::kUseParabolaTerm;
    return slosh;
}

ExecutionModelContract manifestExecutionContract() {
    ExecutionModelContract contract;
    contract.schema_version = manifest::kExecutionContractSchemaVersion;
    contract.contract_id = manifest::kContractId;
    contract.contract_hash = manifest::kContractHash;
    contract.dt = manifest::kDt;
    contract.linear.delay_sec = manifest::kLinearDelaySec;
    contract.linear.time_constant_sec = manifest::kLinearTimeConstantSec;
    contract.linear.positive_gain = manifest::kLinearPositiveGain;
    contract.linear.negative_gain = manifest::kLinearNegativeGain;
    contract.linear.deadzone = manifest::kLinearDeadzone;
    contract.linear.output_min = manifest::kLinearOutputMin;
    contract.linear.output_max = manifest::kLinearOutputMax;
    contract.angular.delay_sec = manifest::kAngularDelaySec;
    contract.angular.time_constant_sec = manifest::kAngularTimeConstantSec;
    contract.angular.positive_gain = manifest::kAngularPositiveGain;
    contract.angular.negative_gain = manifest::kAngularNegativeGain;
    contract.angular.deadzone = manifest::kAngularDeadzone;
    contract.angular.output_min = manifest::kAngularOutputMin;
    contract.angular.output_max = manifest::kAngularOutputMax;
    return contract;
}

EmpiricalRecoveryRadii unitRadii(double x = 1.0) {
    EmpiricalRecoveryRadii radii;
    radii.x = x;
    radii.y = 1.0;
    radii.yaw = 1.0;
    radii.v = 1.0;
    radii.omega = 1.0;
    radii.eta_x = 1.0;
    radii.eta_x_dot = 1.0;
    radii.eta_y = 1.0;
    radii.eta_y_dot = 1.0;
    return radii;
}

ExecutionCompatibilityBounds unitBounds() {
    ExecutionCompatibilityBounds bounds;
    bounds.valid = true;
    bounds.linear_actuator_output = 1.0;
    bounds.angular_actuator_output = 1.0;
    bounds.linear_pending_commands.assign(
        static_cast<std::size_t>(manifest::kLinearBufferCount), 1.0);
    bounds.angular_pending_commands.assign(
        static_cast<std::size_t>(manifest::kAngularBufferCount), 1.0);
    return bounds;
}

ExecutionAugmentedState zeroExecution(std::size_t index) {
    ExecutionAugmentedState execution;
    execution.valid = true;
    execution.stage_index = index;
    execution.robot.x = kPathLength;
    execution.linear.pending_commands.assign(
        static_cast<std::size_t>(manifest::kLinearBufferCount), 0.0);
    execution.angular.pending_commands.assign(
        static_cast<std::size_t>(manifest::kAngularBufferCount), 0.0);
    return execution;
}

std::vector<PhaseNominalSample> zeroSamples(double radius_zero,
                                            double radius_one) {
    std::vector<PhaseNominalSample> samples(kArtifactSize);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        PhaseNominalSample& sample = samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) * manifest::kDt;
        sample.s = kPathLength;
        sample.x = kPathLength;
        sample.radii = unitRadii(index == 0
                                     ? radius_zero
                                     : (index == 1 ? radius_one : 1.0));
        sample.augmented_execution_valid = true;
        sample.augmented_execution = zeroExecution(index);
        sample.execution_bounds = unitBounds();
    }
    return samples;
}

std::map<std::string, std::string> v3Metadata(
    const std::vector<PhaseNominalSample>& samples) {
    const BoundedTrackingRecoveryPolicyParams recovery =
        boundedTrackingRecoveryPolicyV1Params();
    const SloshModelParams slosh_params = manifestSlosh();
    SloshDynamics slosh;
    EXPECT_TRUE(slosh.configure(slosh_params));
    const ExecutionModelContract execution = manifestExecutionContract();
    std::map<std::string, std::string> metadata = {
        {"schema", "phase_rejoin_empirical_augmented_v3"},
        {"evidence_level", "empirical_held_out"},
        {"source", "tail_commit_integration_fixture"},
        {"contract_id", "tail_commit_integration_v3"},
        {"frame_id", "map"},
        {"dt", number(manifest::kDt)},
        {"path_length", number(kPathLength)},
        {"terminal_contract", "stop_settle_zero_hold_v1"},
        {"recovery_contract", recovery.contract_id},
        {"recovery_policy_longitudinal_position_gain",
         number(recovery.longitudinal_position_gain)},
        {"recovery_policy_lateral_position_gain",
         number(recovery.lateral_position_gain)},
        {"recovery_policy_yaw_gain", number(recovery.yaw_gain)},
        {"recovery_policy_linear_velocity_gain",
         number(recovery.linear_velocity_gain)},
        {"recovery_policy_angular_velocity_gain",
         number(recovery.angular_velocity_gain)},
        {"recovery_policy_max_residual_v",
         number(recovery.max_residual_v)},
        {"recovery_policy_max_residual_omega",
         number(recovery.max_residual_omega)},
        {"recovery_policy_published_linear_min",
         number(recovery.published_linear_min)},
        {"recovery_policy_published_linear_max",
         number(recovery.published_linear_max)},
        {"recovery_policy_published_angular_min",
         number(recovery.published_angular_min)},
        {"recovery_policy_published_angular_max",
         number(recovery.published_angular_max)},
        {"terminal_zero_hold_steps", "11"},
        {"terminal_eta_norm_max", "1.0"},
        {"terminal_eta_dot_norm_max", "1.0"},
        {"two_zeta_omega_n", number(
             2.0 * slosh_params.damping_ratio * slosh.omegaN())},
        {"omega_n_sq", number(slosh.omegaN() * slosh.omegaN())},
        {"kappa_x", "1.0"},
        {"kappa_y", "1.0"},
        {"dynamics_tolerance", number(
             manifest::kPublishedConsistencyTolerance)},
        {"execution_contract_id", execution.contract_id},
        {"execution_contract_hash", execution.contract_hash},
        {"execution_state_width", std::to_string(manifest::kStateCount)},
        {"execution_linear_buffer_count",
         std::to_string(manifest::kLinearBufferCount)},
        {"execution_angular_buffer_count",
         std::to_string(manifest::kAngularBufferCount)},
        {"parameter_schema_version",
         std::to_string(manifest::kParameterSchemaVersion)},
        {"parameter_schema_id", manifest::kParameterSchemaId},
        {"parameter_schema_hash", manifest::kParameterSchemaHash},
        {"recovery_artifact_hash", std::string(64, '0')},
        {"execution_compatibility_contract",
         manifest::kExecutionCompatibilityContract},
    };
    metadata["recovery_artifact_hash"] =
        NominalSequenceArtifact::canonicalRecoveryArtifactHash(
            metadata, samples);
    return metadata;
}

NominalSequenceArtifact makeArtifact(double radius_zero = 1.0,
                                     double radius_one = 1.0) {
    const std::vector<PhaseNominalSample> samples =
        zeroSamples(radius_zero, radius_one);
    NominalSequenceArtifact artifact;
    const NominalArtifactLoadResult result = artifact.assignValidated(
        v3Metadata(samples), samples, "<tail-commit-integration>");
    EXPECT_TRUE(result.success) << result.status << ": " << result.detail;
    return artifact;
}

ReferencePath fixtureReference() {
    std::vector<TrajectoryPoint> points(2);
    points[1].x = kPathLength;
    ReferencePath reference;
    reference.setPoints(points, "map");
    return reference;
}

PhaseRejoinRuntimeContract fixtureRuntime(
    const NominalSequenceArtifact& artifact) {
    const ExecutionModelContract execution = manifestExecutionContract();
    const SloshModelParams slosh_params = manifestSlosh();
    SloshDynamics slosh;
    EXPECT_TRUE(slosh.configure(slosh_params));
    PhaseRejoinRuntimeContract runtime;
    runtime.liquid_model_configured = true;
    runtime.slosh_model = slosh_params;
    runtime.dt = manifest::kDt;
    runtime.two_zeta_omega_n =
        2.0 * slosh_params.damping_ratio * slosh.omegaN();
    runtime.omega_n_sq = slosh.omegaN() * slosh.omegaN();
    runtime.kappa_x = runtime.kappa_y = 1.0;
    runtime.min_command_v = manifest::kLinearOutputMin;
    runtime.max_command_v = manifest::kLinearOutputMax;
    runtime.max_abs_command_omega = manifest::kAngularOutputMax;
    runtime.delay_augmented_solver_requested = true;
    runtime.execution_contract_id = execution.contract_id;
    runtime.execution_contract_hash = execution.contract_hash;
    runtime.execution_state_width = manifest::kStateCount;
    runtime.linear_buffer_count = manifest::kLinearBufferCount;
    runtime.angular_buffer_count = manifest::kAngularBufferCount;
    runtime.solver_control_width = manifest::kControlCount;
    runtime.execution_front_steps = manifest::kExecutionFrontSteps;
    runtime.solver_horizon_steps = manifest::kHorizonSteps;
    runtime.max_published_acceleration = manifest::kAccelerationMax;
    runtime.max_published_angular_acceleration =
        manifest::kAngularAccelerationMax;
    runtime.parameter_schema_version = manifest::kParameterSchemaVersion;
    runtime.parameter_schema_id = manifest::kParameterSchemaId;
    runtime.parameter_schema_hash = manifest::kParameterSchemaHash;
    runtime.recovery_artifact_hash =
        artifact.metadata().recovery_artifact_hash;
    runtime.execution_compatibility_contract =
        manifest::kExecutionCompatibilityContract;
    runtime.solver_capabilities = manifest::kCapabilities;
    runtime.required_solver_capabilities = manifest::kFormalRequiredCapabilities;
    runtime.delay_augmented_weights.position = 1.0;
    return runtime;
}

PhaseRejoinParams fixtureParams(int max_holds = 2,
                                bool empirical_gate_enforced = true) {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Enforce;
    params.progress_governor_enabled = true;
    params.successor_admission_enabled = true;
    params.tail_commit_enabled = true;
    params.max_consecutive_phase_holds = max_holds;
    params.empirical_gate_enforced = empirical_gate_enforced;
    params.required_contract_id = "tail_commit_integration_v3";
    params.required_frame_id = "map";
    params.liquid_horizon_steps = manifest::kLiquidHorizonSteps;
    params.max_residual_v = 0.08;
    params.max_residual_omega = 0.20;
    return params;
}

ExecutionHorizonContext fixtureHorizon(const ExecutionAugmentedState& initial) {
    ExecutionHorizonContext horizon;
    horizon.active = true;
    horizon.contract = manifestExecutionContract();
    horizon.initial_state = initial;
    horizon.initial_epoch_ns = secondsToNanoseconds(10.0);
    horizon.execution_front_steps = manifest::kExecutionFrontSteps;
    horizon.liquid_horizon_steps = manifest::kLiquidHorizonSteps;
    horizon.horizon_steps = manifest::kHorizonSteps;
    horizon.physical_front_epoch_ns = horizon.initial_epoch_ns;
    horizon.grid_front_epoch_ns = addSeconds(
        horizon.initial_epoch_ns,
        manifest::kExecutionFrontSteps * manifest::kDt);
    horizon.terminal_epoch_ns = addSeconds(
        horizon.initial_epoch_ns,
        manifest::kHorizonSteps * manifest::kDt);
    return horizon;
}

PhaseRejoinCoordinator configuredCoordinator(
    double radius_zero = 1.0, double radius_one = 1.0,
    int max_holds = 2, bool empirical_gate_enforced = true) {
    const NominalSequenceArtifact artifact =
        makeArtifact(radius_zero, radius_one);
    PhaseRejoinCoordinator coordinator;
    std::string error;
    EXPECT_TRUE(coordinator.configure(
        fixtureParams(max_holds, empirical_gate_enforced), error))
        << error;
    EXPECT_TRUE(coordinator.setArtifact(artifact, error)) << error;
    EXPECT_TRUE(coordinator.validateRuntimeContract(
        fixtureRuntime(artifact), fixtureReference(), error)) << error;
    return coordinator;
}

PhaseRejoinPreparation prepareAt(PhaseRejoinCoordinator& coordinator,
                                 std::size_t index) {
    const ExecutionAugmentedState execution = zeroExecution(index);
    const ExecutionHorizonContext horizon = fixtureHorizon(execution);
    return coordinator.prepare(
        execution.robot, execution.slosh,
        manifest::kExecutionFrontSteps, manifest::kHorizonSteps,
        static_cast<double>(index) * manifest::kDt,
        false, true, &execution, &horizon);
}

PhaseRejoinPreparation prepareWithExecution(
    PhaseRejoinCoordinator& coordinator, std::size_t index,
    const ExecutionAugmentedState& execution) {
    const ExecutionHorizonContext horizon = fixtureHorizon(execution);
    return coordinator.prepare(
        execution.robot, execution.slosh,
        manifest::kExecutionFrontSteps, manifest::kHorizonSteps,
        static_cast<double>(index) * manifest::kDt,
        false, true, &execution, &horizon);
}

PhaseSolveView solveAt(const NominalSequenceArtifact& artifact,
                       std::size_t index,
                       double successor_x = kPathLength) {
    PhaseSolveView solve;
    solve.cmd_v = 0.0;
    solve.cmd_omega = 0.0;
    solve.terminal_state_available = true;
    solve.terminal_robot.x = kPathLength;
    solve.terminal_execution_state_available = true;
    solve.terminal_execution = zeroExecution(
        index + static_cast<std::size_t>(manifest::kHorizonSteps));
    solve.current_execution_state_available = true;
    solve.current_execution = zeroExecution(index);
    solve.successor_execution_state_available = true;
    solve.successor_execution = zeroExecution(index + 1);
    solve.successor_execution.robot.x = successor_x;
    (void)artifact;
    return solve;
}

class TailFakeSolverSession : public SolverSession {
public:
    bool solve(const SolverInput& input, SolverOutput& output) override {
        ++calls;
        last_input = input;
        output = next_output;
        return solve_return;
    }

    int calls = 0;
    bool solve_return = true;
    SolverInput last_input;
    SolverOutput next_output;
};

class TailFakeCommandSink : public ICommandSink {
public:
    StampNs publicationTimeNs() override { return now_ns; }

    PublicationReceipt publish(const FinalCommand& command) override {
        ++calls;
        last_command = command;
        PublicationReceipt receipt;
        receipt.cycle_id = mismatch_receipt
            ? command.cycle_id + 1
            : command.cycle_id;
        receipt.attempted = true;
        receipt.delivered = deliver;
        receipt.actual_publish_stamp_ns = deliver ? now_ns : 0;
        receipt.command = command.command;
        receipt.status = deliver ? "FAKE_DELIVERED" : "FAKE_FAILED";
        return receipt;
    }

    StampNs now_ns = secondsToNanoseconds(10.0);
    bool deliver = true;
    bool mismatch_receipt = false;
    int calls = 0;
    FinalCommand last_command;
};

SolverOutput tailResidualOutput(std::size_t index) {
    SolverOutput output;
    output.success = true;
    output.status = "OK";
    output.cmd_v = 0.0;
    output.cmd_omega = 0.0;
    output.delay_augmented_execution_solution = true;
    output.initial_execution_state = zeroExecution(index);
    output.successor_execution_state = zeroExecution(index + 1);
    output.terminal_execution_state = zeroExecution(
        index + static_cast<std::size_t>(manifest::kHorizonSteps));
    output.predicted_horizon.valid = true;
    output.predicted_horizon.states.resize(
        static_cast<std::size_t>(manifest::kHorizonSteps) + 1u);
    for (HorizonStateDebug& state : output.predicted_horizon.states) {
        state.x = kPathLength;
    }
    return output;
}

class TailEngineFixture {
public:
    TailEngineFixture() : engine(solver) {
        std::string error;
        EXPECT_TRUE(engine.configurePhaseRejoin(
            fixtureParams(), error)) << error;
        EXPECT_TRUE(engine.configureSafety(safety, error)) << error;

        CommandPipelineConfig pipeline;
        pipeline.linear_accel_limit_enable = false;
        pipeline.angular_limit_enable = false;
        EXPECT_TRUE(engine.configureCommandPipeline(pipeline, error))
            << error;

        PublishLatencyModelConfig latency;
        latency.enabled = true;
        latency.estimated_dc_sec = 0.05;
        EXPECT_TRUE(engine.configurePublishLatency(latency, error))
            << error;
        history.configure(2.0);
    }

    bool loadFixtureArtifact(std::string& error) {
        static unsigned int serial = 0;
        const std::string path = "/tmp/spmpc_tail_commit_engine_" +
            std::to_string(static_cast<long long>(::getpid())) + "_" +
            std::to_string(serial++) + ".csv";
        const NominalSequenceArtifact artifact = makeArtifact();
        const NominalArtifactLoadResult written =
            artifact.writeCanonicalCsv(path, true);
        if (!written.success) {
            error = written.status + ": " + written.detail;
            return false;
        }
        const NominalArtifactLoadResult loaded =
            engine.loadPhaseRejoinArtifact(path);
        std::remove(path.c_str());
        if (!loaded.success) {
            error = loaded.status + ": " + loaded.detail;
            return false;
        }
        if (!engine.validatePhaseRejoinRuntimeContract(
                fixtureRuntime(artifact), fixtureReference(), error)) {
            return false;
        }
        return true;
    }

    ControlCycleRequest requestAt(std::size_t index,
                                  double execution_omega = 0.0) {
        ControlCycleRequest request;
        request.cycle_id = static_cast<std::uint64_t>(index + 1u);
        request.cycle_start_ns = secondsToNanoseconds(
            10.0 + static_cast<double>(index) * 0.1);
        request.period_sec = 0.1;
        request.control_period_sec = 0.1;
        request.prediction_valid = true;
        request.prediction_status = "OK";
        // The 15D solver origin is the aligned execution state itself, not
        // the pre-alignment execution-front snapshot.
        request.solver_origin_at_execution_front = false;
        request.execution_front_steps = manifest::kExecutionFrontSteps;
        request.command_sink = &sink;
        request.command_history = &history;
        request.solver_input.dt = manifest::kDt;
        request.solver_input.horizon_steps = manifest::kHorizonSteps;

        ExecutionAugmentedState initial = zeroExecution(index);
        initial.robot.omega = execution_omega;
        request.solver_input.execution_horizon = fixtureHorizon(initial);
        CycleTimingContract timing;
        timing.cycle_id = request.cycle_id;
        timing.cycle_start_stamp_ns = request.cycle_start_ns;
        timing.control_period_sec = request.control_period_sec;
        request.publish_epoch_estimate = engine.estimatePublishEpoch(timing);
        request.solver_input.execution_horizon.initial_epoch_ns =
            request.publish_epoch_estimate.expected_publish_stamp_ns;
        request.solver_input.execution_horizon.physical_front_epoch_ns =
            request.publish_epoch_estimate.expected_publish_stamp_ns;
        request.solver_input.execution_horizon.grid_front_epoch_ns = addSeconds(
            request.solver_input.execution_horizon.initial_epoch_ns,
            manifest::kExecutionFrontSteps * manifest::kDt);
        request.solver_input.execution_horizon.terminal_epoch_ns = addSeconds(
            request.solver_input.execution_horizon.initial_epoch_ns,
            manifest::kHorizonSteps * manifest::kDt);
        request.execution_front_robot = initial.robot;
        request.execution_front_slosh = initial.slosh;
        sink.now_ns = request.publish_epoch_estimate.expected_publish_stamp_ns;
        return request;
    }

    ControlCycleResult stepAt(std::size_t index,
                              double execution_omega = 0.0) {
        solver.next_output = tailResidualOutput(index);
        return engine.step(requestAt(index, execution_omega));
    }

    void primeToTail() {
        std::string error;
        ASSERT_TRUE(loadFixtureArtifact(error)) << error;
        for (std::size_t index = 0; index < kMaxTailIndex; ++index) {
            const ControlCycleResult result = stepAt(index);
            ASSERT_TRUE(result.solve_returned) << index;
            ASSERT_TRUE(result.phase_committed) << index << ": "
                << result.phase_decision.status;
            ASSERT_EQ(result.phase_preparation.candidate.current_index,
                      index);
        }
        ASSERT_EQ(solver.calls, static_cast<int>(kMaxTailIndex));
        ASSERT_EQ(engine.phaseRejoinCoordinator().progressIndex(),
                  kMaxTailIndex);
    }

    TailFakeSolverSession solver;
    TailFakeCommandSink sink;
    CommandHistoryBuffer history;
    ControlCycleEngine engine;
    SafetySupervisorConfig safety;
};

TEST(PhaseRejoinTailCommitIntegration,
     AdvanceIsProposedWithoutMutationAndCommitMovesExactlyOne) {
    PhaseRejoinCoordinator coordinator = configuredCoordinator();
    const NominalSequenceArtifact& artifact = coordinator.artifact();
    const PhaseRejoinPreparation preparation = prepareAt(coordinator, 0);
    ASSERT_TRUE(preparation.ready) << preparation.status;
    ASSERT_EQ(preparation.candidate.current_index, 0u);

    const PhaseRejoinDecision decision = coordinator.decide(
        preparation, artifact.sample(0)->augmented_execution.robot,
        SloshState{}, true, solveAt(artifact, 0));
    ASSERT_TRUE(decision.evaluated);
    EXPECT_TRUE(decision.successor_advance_admitted);
    EXPECT_EQ(decision.phase_progress_action, "ADVANCE");
    EXPECT_EQ(coordinator.progressIndex(), 0u);
    EXPECT_FALSE(coordinator.haveAcceptedIndex());

    ASSERT_TRUE(coordinator.commit(preparation, decision));
    EXPECT_EQ(coordinator.progressIndex(), 1u);
    EXPECT_EQ(coordinator.acceptedIndex(), 1u);
}

TEST(PhaseRejoinTailCommitIntegration,
     RejectedAdvanceWithAcceptedCurrentProposesHold) {
    PhaseRejoinCoordinator coordinator = configuredCoordinator(1.0, 0.01);
    const NominalSequenceArtifact& artifact = coordinator.artifact();
    const PhaseRejoinPreparation preparation = prepareAt(coordinator, 0);
    ASSERT_TRUE(preparation.ready) << preparation.status;
    const PhaseSolveView solve = solveAt(artifact, 0, kPathLength + 0.02);
    const PhaseRejoinDecision decision = coordinator.decide(
        preparation, artifact.sample(0)->augmented_execution.robot,
        SloshState{}, true, solve);

    EXPECT_FALSE(decision.successor_advance_admitted);
    EXPECT_TRUE(decision.successor_hold_admitted);
    EXPECT_TRUE(decision.phase_progress_decision_valid);
    EXPECT_EQ(decision.phase_progress_action, "HOLD");
    EXPECT_EQ(decision.phase_progress_next_index, 0u);
    EXPECT_EQ(coordinator.progressIndex(), 0u);
    ASSERT_TRUE(coordinator.commit(preparation, decision));
    EXPECT_EQ(coordinator.progressIndex(), 0u);
}

TEST(PhaseRejoinTailCommitIntegration,
     RejectedSuccessorsRejectResidualAndSelectRecoveryWithTailRequest) {
    PhaseRejoinCoordinator coordinator = configuredCoordinator(0.01, 0.01);
    const NominalSequenceArtifact& artifact = coordinator.artifact();
    const PhaseRejoinPreparation preparation = prepareAt(coordinator, 0);
    ASSERT_TRUE(preparation.ready) << preparation.status;
    const PhaseSolveView solve = solveAt(artifact, 0, kPathLength + 0.02);
    const PhaseRejoinDecision decision = coordinator.decide(
        preparation, artifact.sample(0)->augmented_execution.robot,
        SloshState{}, true, solve);

    EXPECT_FALSE(decision.successor_advance_admitted);
    EXPECT_FALSE(decision.successor_hold_admitted);
    EXPECT_FALSE(decision.phase_progress_decision_valid);
    EXPECT_EQ(decision.phase_progress_action, "REJECT");
    EXPECT_TRUE(decision.command_intervened);
    EXPECT_TRUE(decision.recovery_command_used);
    EXPECT_TRUE(decision.tail_command_used);
    EXPECT_TRUE(decision.tail_commit_requested);
    EXPECT_EQ(decision.output_cmd_v, preparation.recovery_cmd_v);
    EXPECT_EQ(decision.output_cmd_omega, preparation.recovery_cmd_omega);
    EXPECT_FALSE(coordinator.commit(preparation, decision));
    EXPECT_EQ(coordinator.progressIndex(), 0u);
}

TEST(PhaseRejoinTailCommitIntegration,
     HoldLimitRejectsResidualAndSelectsRecoveryWithTailRequest) {
    PhaseRejoinCoordinator coordinator =
        configuredCoordinator(1.0, 0.01, 0);
    const NominalSequenceArtifact& artifact = coordinator.artifact();
    const PhaseRejoinPreparation preparation = prepareAt(coordinator, 0);
    ASSERT_TRUE(preparation.ready) << preparation.status;
    const PhaseSolveView solve = solveAt(artifact, 0, kPathLength + 0.02);
    const PhaseRejoinDecision decision = coordinator.decide(
        preparation, artifact.sample(0)->augmented_execution.robot,
        SloshState{}, true, solve);

    EXPECT_FALSE(decision.successor_advance_admitted);
    EXPECT_TRUE(decision.successor_hold_admitted);
    EXPECT_FALSE(decision.phase_progress_decision_valid);
    EXPECT_EQ(decision.phase_progress_action, "REJECT");
    EXPECT_TRUE(decision.command_intervened);
    EXPECT_TRUE(decision.recovery_command_used);
    EXPECT_TRUE(decision.tail_commit_requested);
    EXPECT_FALSE(coordinator.commit(preparation, decision));
    EXPECT_EQ(coordinator.progressIndex(), 0u);
}

TEST(PhaseRejoinTailCommitIntegration,
     RequestsTailOnlyAfterLastCompleteSolverWindowIsAdmitted) {
    PhaseRejoinCoordinator coordinator = configuredCoordinator();
    const NominalSequenceArtifact& artifact = coordinator.artifact();
    constexpr std::size_t max_current = kArtifactSize -
        static_cast<std::size_t>(manifest::kHorizonSteps) - 1u;

    for (std::size_t index = 0; index < max_current; ++index) {
        const PhaseRejoinPreparation preparation = prepareAt(coordinator, index);
        ASSERT_TRUE(preparation.ready) << preparation.status << " at " << index;
        ASSERT_FALSE(preparation.tail_commit_armed) << index;
        const PhaseRejoinDecision decision = coordinator.decide(
            preparation, artifact.sample(index)->augmented_execution.robot,
            SloshState{}, true, solveAt(artifact, index));
        ASSERT_EQ(decision.phase_progress_action, "ADVANCE") << index;
        ASSERT_TRUE(coordinator.commit(preparation, decision)) << index;
    }

    const PhaseRejoinPreparation tail = prepareAt(coordinator, max_current);
    ASSERT_TRUE(tail.ready) << tail.status;
    EXPECT_FALSE(tail.tail_commit_armed);
    EXPECT_EQ(tail.tail_artifact_index, max_current);
    EXPECT_EQ(tail.candidate.current_index, max_current);
    EXPECT_EQ(tail.candidate.terminal_index, kArtifactSize - 1u);
    EXPECT_TRUE(tail.solver_context.delay_augmented.active);
    EXPECT_EQ(tail.solver_context.delay_augmented.horizon_steps,
              manifest::kHorizonSteps);
    EXPECT_DOUBLE_EQ(tail.residual_authority_alpha, 0.0);
    EXPECT_DOUBLE_EQ(tail.solver_context.residual_authority_alpha, 0.0);

    const PhaseRejoinDecision decision = coordinator.decide(
        tail, artifact.sample(max_current)->augmented_execution.robot,
        SloshState{}, true, solveAt(artifact, max_current));
    EXPECT_TRUE(decision.phase_progress_decision_valid);
    EXPECT_EQ(decision.phase_progress_action, "COMPLETE");
    EXPECT_TRUE(decision.command_contract_consistent);
    EXPECT_TRUE(decision.tail_command_used);
    EXPECT_TRUE(decision.tail_commit_requested);
    EXPECT_EQ(decision.tail_artifact_index, max_current);
    EXPECT_FALSE(coordinator.commit(tail, decision));
}

TEST(PhaseRejoinTailCommitIntegration,
     ResidualAuthorityIsFullThenTapersBeforeTail) {
    PhaseRejoinCoordinator coordinator = configuredCoordinator();
    const NominalSequenceArtifact& artifact = coordinator.artifact();
    constexpr std::size_t max_current = kMaxTailIndex;

    const PhaseRejoinPreparation distant = prepareAt(coordinator, 0);
    ASSERT_TRUE(distant.ready) << distant.status;
    EXPECT_DOUBLE_EQ(distant.residual_authority_alpha, 1.0);
    EXPECT_DOUBLE_EQ(
        distant.solver_context.delay_augmented.max_residual_v, 0.08);

    // The governed selector intentionally pins to the committed cursor, so
    // advance a separate coordinator to the near-tail cursor first.
    PhaseRejoinCoordinator near_tail_coordinator = configuredCoordinator();
    for (std::size_t index = 0; index < max_current - 1u; ++index) {
        const PhaseRejoinPreparation step = prepareAt(
            near_tail_coordinator, index);
        ASSERT_TRUE(step.ready) << step.status << " at " << index;
        const PhaseRejoinDecision step_decision = near_tail_coordinator.decide(
            step, artifact.sample(index)->augmented_execution.robot,
            SloshState{}, true, solveAt(artifact, index));
        ASSERT_TRUE(near_tail_coordinator.commit(step, step_decision))
            << index;
    }
    const PhaseRejoinPreparation near_tail = prepareAt(
        near_tail_coordinator, max_current - 1u);
    ASSERT_TRUE(near_tail.ready) << near_tail.status;
    const double expected_alpha = 1.0 /
        static_cast<double>(manifest::kHorizonSteps);
    EXPECT_NEAR(near_tail.residual_authority_alpha, expected_alpha, 1e-12);
    EXPECT_NEAR(
        near_tail.solver_context.delay_augmented.max_residual_v,
        0.08 * expected_alpha, 1e-12);
    EXPECT_NEAR(
        near_tail.solver_context.delay_augmented.max_residual_omega,
        0.20 * expected_alpha, 1e-12);

    const PhaseRejoinDecision decision = near_tail_coordinator.decide(
        near_tail,
        artifact.sample(max_current - 1u)->augmented_execution.robot,
        SloshState{}, true, solveAt(artifact, max_current - 1u));
    EXPECT_DOUBLE_EQ(decision.residual_authority_alpha, expected_alpha);

    PhaseSolveView over_bound = solveAt(artifact, max_current - 1u);
    over_bound.cmd_v = near_tail.nominal_cmd_v + 0.02;
    const PhaseRejoinDecision rejected = near_tail_coordinator.decide(
        near_tail,
        artifact.sample(max_current - 1u)->augmented_execution.robot,
        SloshState{}, true, over_bound);
    EXPECT_TRUE(rejected.controlled_stop_used);
    EXPECT_EQ(rejected.status,
              "ENFORCE_SOLVER_COMMAND_CONTRACT_VIOLATION");
}

TEST(PhaseRejoinTailCommitIntegration,
     C4EmpiricalBoundaryShrinksAuthorityButC3MonitorDoesNot) {
    const double current_error = std::sqrt(0.75);
    ExecutionAugmentedState execution = zeroExecution(0);
    execution.robot.x += current_error;

    PhaseRejoinCoordinator c4 = configuredCoordinator();
    const PhaseRejoinPreparation c4_preparation = prepareWithExecution(
        c4, 0, execution);
    ASSERT_TRUE(c4_preparation.ready) << c4_preparation.status;
    EXPECT_NEAR(c4_preparation.residual_authority_alpha, 0.25, 1e-12);
    EXPECT_NEAR(
        c4_preparation.solver_context.delay_augmented.max_residual_v,
        0.08 * 0.25, 1e-12);

    PhaseRejoinCoordinator c3 = configuredCoordinator(
        1.0, 1.0, 2, false);
    const PhaseRejoinPreparation c3_preparation = prepareWithExecution(
        c3, 0, execution);
    ASSERT_TRUE(c3_preparation.ready) << c3_preparation.status;
    EXPECT_DOUBLE_EQ(c3_preparation.residual_authority_alpha, 1.0);
    EXPECT_DOUBLE_EQ(
        c3_preparation.solver_context.delay_augmented.max_residual_v, 0.08);
}

TEST(ControlCycleEngineTailCommitIntegration,
     ConsistentTailPublicationCommitsAndAdvancesAfterSolverAdmission) {
    TailEngineFixture fixture;
    fixture.primeToTail();

    const int solver_calls_before_tail = fixture.solver.calls;
    const ControlCycleResult first = fixture.stepAt(kMaxTailIndex);
    ASSERT_EQ(solver_calls_before_tail + 1, fixture.solver.calls);
    EXPECT_FALSE(first.phase_preparation.tail_commit_armed);
    EXPECT_TRUE(first.phase_decision.tail_command_used);
    EXPECT_FALSE(first.phase_decision.recovery_command_used);
    EXPECT_TRUE(first.publication.receipt_consistent);
    EXPECT_TRUE(first.publication.history_committed);
    EXPECT_TRUE(first.tail_publication_observed);
    EXPECT_TRUE(first.tail_commit.accepted);
    EXPECT_EQ(first.tail_commit.state, TailCommitState::Committed);
    EXPECT_EQ(first.tail_commit.anchor_index, kMaxTailIndex);
    EXPECT_EQ(first.tail_commit.cursor, kMaxTailIndex + 1u);
    EXPECT_EQ(fixture.engine.tailCommitState(), TailCommitState::Committed);

    const ControlCycleResult second = fixture.stepAt(kMaxTailIndex + 1u);
    EXPECT_EQ(fixture.solver.calls, solver_calls_before_tail + 1);
    EXPECT_FALSE(second.solve_returned);
    EXPECT_TRUE(second.phase_preparation.tail_committed_mode);
    EXPECT_EQ(second.phase_preparation.tail_artifact_index,
              kMaxTailIndex + 1u);
    EXPECT_TRUE(second.phase_decision.tail_command_used);
    EXPECT_TRUE(second.publication.receipt_consistent);
    EXPECT_TRUE(second.publication.history_committed);
    EXPECT_TRUE(second.tail_commit.accepted);
    EXPECT_EQ(second.tail_commit.state, TailCommitState::Committed);
    EXPECT_EQ(second.tail_commit.anchor_index, kMaxTailIndex);
    EXPECT_EQ(second.tail_commit.cursor, kMaxTailIndex + 2u);

    // A transient empirical excursion after commit is recorded but must not
    // revoke the already validated tail when execution contracts remain good.
    ControlCycleRequest excursion = fixture.requestAt(kMaxTailIndex + 2u);
    excursion.solver_input.execution_horizon.initial_state.robot.x += 1.1;
    excursion.execution_front_robot.x += 1.1;
    const ControlCycleResult third = fixture.engine.step(excursion);
    EXPECT_EQ(fixture.solver.calls, solver_calls_before_tail + 1);
    EXPECT_TRUE(third.phase_preparation.tail_committed_mode);
    EXPECT_FALSE(third.phase_decision.current_gate_accepted);
    EXPECT_TRUE(third.phase_decision.current_execution_compatible);
    EXPECT_TRUE(third.phase_decision.tail_command_used);
    EXPECT_NE(third.phase_decision.status,
              "TAIL_CURRENT_ADMISSION_REJECTED_STOP");
    EXPECT_TRUE(third.publication.receipt_consistent);
    EXPECT_TRUE(third.publication.history_committed);
    EXPECT_TRUE(third.tail_commit.accepted);
    EXPECT_EQ(third.tail_commit.state, TailCommitState::Committed);
    EXPECT_EQ(third.tail_commit.cursor, kMaxTailIndex + 3u);

    // Execution compatibility is likewise diagnostic after commitment; it
    // must not revoke the already validated tail by itself.
    ControlCycleRequest execution_excursion =
        fixture.requestAt(kMaxTailIndex + 3u);
    execution_excursion.solver_input.execution_horizon.initial_state
        .linear.pending_commands.front() = 1.1;
    const ControlCycleResult fourth = fixture.engine.step(execution_excursion);
    EXPECT_EQ(fixture.solver.calls, solver_calls_before_tail + 1);
    EXPECT_TRUE(fourth.phase_preparation.tail_committed_mode);
    EXPECT_TRUE(fourth.phase_decision.current_gate_accepted);
    EXPECT_FALSE(fourth.phase_decision.current_execution_compatible);
    EXPECT_TRUE(fourth.phase_decision.tail_command_used);
    EXPECT_NE(fourth.phase_decision.status,
              "TAIL_CURRENT_ADMISSION_REJECTED_STOP");
    EXPECT_TRUE(fourth.publication.receipt_consistent);
    EXPECT_TRUE(fourth.publication.history_committed);
    EXPECT_TRUE(fourth.tail_commit.accepted);
    EXPECT_EQ(fourth.tail_commit.state, TailCommitState::Committed);
    EXPECT_EQ(fourth.tail_commit.cursor, kMaxTailIndex + 4u);
}

TEST(ControlCycleEngineTailCommitIntegration,
     DiagnosticsRetainAbsoluteClockWhenGovernedCursorIsPinned) {
    TailEngineFixture fixture;
    std::string error;
    ASSERT_TRUE(fixture.loadFixtureArtifact(error)) << error;

    const ControlCycleResult first = fixture.stepAt(0);
    ASSERT_TRUE(first.phase_committed);
    const ControlCycleResult second = fixture.stepAt(1);
    ASSERT_TRUE(second.phase_committed);
    EXPECT_GT(second.phase_preparation.phase_clock_index,
              second.phase_preparation.candidate.clock_index);
    EXPECT_EQ(second.phase_debug.clock_index,
              second.phase_preparation.phase_clock_index);
    EXPECT_EQ(second.phase_decision.phase_progress_clock_index,
              second.phase_preparation.phase_clock_index);
    EXPECT_EQ(second.phase_debug.phase_progress_clock_index,
              second.phase_preparation.phase_clock_index);
    EXPECT_EQ(second.phase_debug.phase_progress_lag_steps,
              second.phase_decision.phase_progress_lag_steps);
}

TEST(ControlCycleEngineTailCommitIntegration,
     AcceptedResidualFallsBackToBackupTailWhenNextHorizonIsUnavailable) {
    TailEngineFixture fixture;
    std::string error;
    ASSERT_TRUE(fixture.loadFixtureArtifact(error)) << error;
    const ControlCycleResult first = fixture.stepAt(0);
    ASSERT_TRUE(first.phase_committed);

    ControlCycleRequest request = fixture.requestAt(1);
    request.solver_input.execution_horizon.horizon_steps =
        manifest::kHorizonSteps - 1;
    fixture.solver.next_output = tailResidualOutput(1);
    const ControlCycleResult backup = fixture.engine.step(request);

    EXPECT_FALSE(backup.solve_returned);
    EXPECT_EQ(fixture.solver.calls, 1);
    EXPECT_EQ(backup.phase_preparation.status, "TAIL_BACKUP_READY");
    EXPECT_EQ(backup.phase_debug.clock_index,
              backup.phase_preparation.phase_clock_index);
    EXPECT_TRUE(backup.phase_preparation.tail_backup_used);
    EXPECT_TRUE(backup.phase_decision.tail_backup_used);
    EXPECT_TRUE(backup.phase_decision.tail_command_used);
    EXPECT_TRUE(backup.phase_decision.recovery_command_used);
    EXPECT_TRUE(backup.publication.receipt_consistent);
    EXPECT_TRUE(backup.publication.history_committed);
    EXPECT_EQ(backup.tail_commit.state, TailCommitState::Committed);
    EXPECT_EQ(backup.tail_commit.anchor_index, 1u);
    EXPECT_EQ(backup.tail_commit.cursor, 2u);
}

TEST(ControlCycleEngineTailCommitIntegration,
     AcceptedResidualFallsBackToBackupTailAfterIntegrityFailure) {
    TailEngineFixture fixture;
    std::string error;
    ASSERT_TRUE(fixture.loadFixtureArtifact(error)) << error;
    const ControlCycleResult first = fixture.stepAt(0);
    ASSERT_TRUE(first.phase_committed);

    ControlCycleRequest request = fixture.requestAt(1);
    fixture.solver.next_output = SolverOutput{};
    fixture.solver.next_output.status = "INTEGRITY_FAILURE";
    fixture.solver.next_output.failure_kind = SolverFailureKind::Integrity;
    const ControlCycleResult backup = fixture.engine.step(request);

    EXPECT_TRUE(backup.solve_returned);
    EXPECT_EQ(fixture.solver.calls, 2);
    EXPECT_EQ(backup.phase_preparation.status, "TAIL_BACKUP_READY");
    EXPECT_TRUE(backup.phase_preparation.tail_backup_used);
    EXPECT_TRUE(backup.phase_decision.tail_backup_used);
    EXPECT_TRUE(backup.phase_decision.tail_command_used);
    EXPECT_TRUE(backup.phase_decision.recovery_command_used);
    EXPECT_TRUE(backup.publication.receipt_consistent);
    EXPECT_TRUE(backup.publication.history_committed);
    EXPECT_TRUE(backup.output.success);
    EXPECT_EQ(backup.tail_commit.state, TailCommitState::Committed);
    EXPECT_EQ(backup.tail_commit.anchor_index, 1u);
    EXPECT_EQ(backup.tail_commit.cursor, 2u);
}

TEST(ControlCycleEngineTailCommitIntegration,
     IntegrityFailureWithoutCommittedBackupRemainsFailClosed) {
    TailEngineFixture fixture;
    fixture.solver.next_output = SolverOutput{};
    fixture.solver.next_output.status = "INTEGRITY_FAILURE";
    fixture.solver.next_output.failure_kind = SolverFailureKind::Integrity;

    const ControlCycleResult first = fixture.engine.step(
        fixture.requestAt(0));
    EXPECT_TRUE(first.solve_returned);
    EXPECT_EQ(fixture.solver.calls, 1);
    EXPECT_FALSE(first.output.success);
    EXPECT_EQ(first.decision.source, CommandSource::PhaseRejoin);
    EXPECT_FALSE(first.decision.accepted);
    EXPECT_DOUBLE_EQ(0.0, first.final_command.linear);
    EXPECT_DOUBLE_EQ(0.0, first.final_command.angular);
    EXPECT_EQ(fixture.engine.tailCommitState(), TailCommitState::Residual);
}

TEST(ControlCycleEngineTailCommitIntegration,
     ReceiptMismatchAbortsTailAndRemainsSticky) {
    TailEngineFixture fixture;
    fixture.primeToTail();
    fixture.sink.mismatch_receipt = true;

    const ControlCycleResult failed = fixture.stepAt(kMaxTailIndex);
    EXPECT_FALSE(failed.publication.receipt_consistent);
    EXPECT_TRUE(failed.publication.history_committed);
    EXPECT_TRUE(failed.tail_publication_observed);
    EXPECT_EQ(failed.tail_commit.state, TailCommitState::Aborted);
    EXPECT_EQ(fixture.engine.tailCommitState(), TailCommitState::Aborted);
    EXPECT_EQ(failed.tail_commit.reason, "ABORTED_PUBLICATION_FAULT");

    fixture.sink.mismatch_receipt = false;
    const int solver_calls_after_abort = fixture.solver.calls;
    const ControlCycleResult sticky = fixture.stepAt(kMaxTailIndex + 1u);
    EXPECT_EQ(fixture.solver.calls, solver_calls_after_abort);
    EXPECT_TRUE(sticky.phase_preparation.tail_aborted);
    EXPECT_EQ(fixture.engine.tailCommitState(), TailCommitState::Aborted);
}

TEST(ControlCycleEngineTailCommitIntegration,
     SafetyOverrideAbortsTailAndResetClearsLatch) {
    TailEngineFixture fixture;
    fixture.primeToTail();

    fixture.safety.tracking.spin_max_duration_sec = 0.05;
    std::string error;
    ASSERT_TRUE(fixture.engine.configureSafety(fixture.safety, error))
        << error;
    const ControlCycleResult overridden = fixture.stepAt(kMaxTailIndex, 0.8);
    EXPECT_TRUE(overridden.safety.blocked);
    EXPECT_TRUE(overridden.safety.tracking_safety_blocked);
    EXPECT_TRUE(overridden.tail_publication_observed);
    EXPECT_EQ(overridden.tail_commit.state, TailCommitState::Aborted);
    EXPECT_EQ(fixture.engine.tailCommitState(), TailCommitState::Aborted);

    const int solver_calls_after_abort = fixture.solver.calls;
    const ControlCycleResult sticky = fixture.stepAt(kMaxTailIndex + 1u);
    EXPECT_EQ(fixture.solver.calls, solver_calls_after_abort);
    EXPECT_TRUE(sticky.phase_preparation.tail_aborted);
    EXPECT_EQ(fixture.engine.tailCommitState(), TailCommitState::Aborted);

    fixture.engine.resetForReference();
    EXPECT_EQ(fixture.engine.tailCommitState(), TailCommitState::Residual);
    EXPECT_FALSE(fixture.engine.phaseRejoinCoordinator().haveAcceptedIndex());
    EXPECT_FALSE(
        fixture.engine.phaseRejoinCoordinator().progressGovernorInitialized());

    const ControlCycleResult after_reset = fixture.stepAt(0);
    EXPECT_EQ(fixture.solver.calls, solver_calls_after_abort + 1);
    EXPECT_FALSE(after_reset.phase_preparation.tail_commit_armed);
    EXPECT_TRUE(after_reset.phase_committed);
    EXPECT_EQ(after_reset.phase_preparation.candidate.current_index, 0u);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
