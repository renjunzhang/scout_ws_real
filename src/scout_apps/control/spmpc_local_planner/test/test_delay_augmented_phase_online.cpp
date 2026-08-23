#include "spmpc_local_planner/controller/control_cycle_engine.h"
#include "spmpc_local_planner/controller/phase_solve_adapter.h"
#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/phase_rejoin/bounded_tracking_recovery_policy.h"
#include "spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_horizon_context_builder.h"
#include "spmpc_local_planner/solver/acados/delay_augmented_phase_parameter_builder.h"
#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"
#include "spmpc_local_planner/solver/api/backend.h"
#include "spmpc_local_planner/solvers/delay_augmented_phase_online_solver.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

constexpr char kNominalContractId[] = "test_augmented_nominal_v3";

std::string number(double value) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << value;
    return out.str();
}

DelayAugmentedPhaseCompiledContract compiledContract() {
    return DelayAugmentedPhaseAcadosSolver::compiledContract();
}

DelayAugmentedPhaseCostWeights costWeights() {
    DelayAugmentedPhaseCostWeights weights;
    weights.position = 1.0;
    weights.yaw = 0.2;
    weights.progress = 0.2;
    weights.v = 1.0;
    weights.omega = 0.1;
    weights.slosh_eta = 1.0;
    weights.slosh_eta_dot = 0.3;
    weights.linear_pending = 1.0;
    weights.angular_pending = 0.1;
    weights.acceleration = 0.1;
    weights.angular_acceleration = 0.1;
    weights.progress_rate = 0.3;
    return weights;
}

ExecutionAugmentedState zeroExecution(std::size_t index) {
    ExecutionAugmentedState state;
    state.valid = true;
    state.stage_index = index;
    state.linear.pending_commands.assign(
        static_cast<std::size_t>(manifest::kLinearBufferCount), 0.0);
    state.angular.pending_commands.assign(
        static_cast<std::size_t>(manifest::kAngularBufferCount), 0.0);
    return state;
}

ExecutionHorizonContext horizonFromExecution(
    const ExecutionAugmentedState& execution) {
    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    ExecutionHorizonContext horizon;
    horizon.active = true;
    horizon.contract = compiled.execution;
    horizon.initial_state = execution;
    horizon.initial_epoch_ns = secondsToNanoseconds(10.0);
    horizon.execution_front_steps = manifest::kExecutionFrontSteps;
    horizon.liquid_horizon_steps = manifest::kLiquidHorizonSteps;
    horizon.horizon_steps = manifest::kHorizonSteps;
    horizon.physical_front_epoch_ns = addSeconds(
        horizon.initial_epoch_ns,
        std::max(compiled.execution.linear.delay_sec,
                 compiled.execution.angular.delay_sec));
    horizon.grid_front_epoch_ns = addSeconds(
        horizon.initial_epoch_ns,
        manifest::kExecutionFrontSteps * manifest::kDt);
    horizon.terminal_epoch_ns = addSeconds(
        horizon.initial_epoch_ns,
        manifest::kHorizonSteps * manifest::kDt);
    return horizon;
}

ExecutionCompatibilityBounds unitExecutionBounds(double bound = 1.0) {
    ExecutionCompatibilityBounds bounds;
    bounds.valid = true;
    bounds.linear_actuator_output = bound;
    bounds.angular_actuator_output = bound;
    bounds.linear_pending_commands.assign(
        static_cast<std::size_t>(manifest::kLinearBufferCount), bound);
    bounds.angular_pending_commands.assign(
        static_cast<std::size_t>(manifest::kAngularBufferCount), bound);
    return bounds;
}

EmpiricalRecoveryRadii unitRadii() {
    EmpiricalRecoveryRadii radii;
    radii.x = 1.0;
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

std::map<std::string, std::string> artifactMetadata(
    const std::string& evidence = "empirical_held_out") {
    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    SloshDynamics slosh;
    EXPECT_TRUE(slosh.configure(compiled.slosh));
    const double omega_n = slosh.omegaN();
    const BoundedTrackingRecoveryPolicyParams recovery =
        boundedTrackingRecoveryPolicyV1Params();
    return {
        {"schema", "phase_rejoin_empirical_augmented_v3"},
        {"evidence_level", evidence},
        {"source", "unit_test_augmented_nominal"},
        {"contract_id", kNominalContractId},
        {"frame_id", "map"},
        {"dt", number(manifest::kDt)},
        {"path_length", "0.09"},
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
        {"two_zeta_omega_n",
         number(2.0 * compiled.slosh.damping_ratio * omega_n)},
        {"omega_n_sq", number(omega_n * omega_n)},
        {"kappa_x", "1.0"},
        {"kappa_y", "1.0"},
        {"dynamics_tolerance", number(
            manifest::kPublishedConsistencyTolerance)},
        {"execution_contract_id", compiled.execution.contract_id},
        {"execution_contract_hash", compiled.execution.contract_hash},
        {"execution_state_width", std::to_string(compiled.state_width)},
        {"execution_linear_buffer_count",
         std::to_string(manifest::kLinearBufferCount)},
        {"execution_angular_buffer_count",
         std::to_string(manifest::kAngularBufferCount)},
        {"parameter_schema_version",
         std::to_string(compiled.parameter_schema_version)},
        {"parameter_schema_id", compiled.parameter_schema_id},
        {"parameter_schema_hash", compiled.parameter_schema_hash},
        {"recovery_artifact_hash", std::string(64, '0')},
        {"execution_compatibility_contract",
         compiled.execution_compatibility_contract},
    };
}

std::vector<PhaseNominalSample> zeroNominalSamples(
    double execution_bound = 1.0) {
    std::vector<PhaseNominalSample> samples(24);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        PhaseNominalSample& sample = samples[index];
        sample.index = index;
        sample.t = static_cast<double>(index) * manifest::kDt;
        sample.s = 0.09;
        sample.x = 0.09;
        sample.radii = unitRadii();
        sample.augmented_execution_valid = true;
        sample.augmented_execution = zeroExecution(index);
        sample.execution_bounds = unitExecutionBounds(execution_bound);
    }
    return samples;
}

std::string recoveryArtifactHash(
    const std::string& evidence = "empirical_held_out",
    double execution_bound = 1.0) {
    const std::vector<PhaseNominalSample> samples =
        zeroNominalSamples(execution_bound);
    return NominalSequenceArtifact::canonicalRecoveryArtifactHash(
        artifactMetadata(evidence), samples);
}

NominalSequenceArtifact augmentedArtifact(
    const std::string& evidence = "empirical_held_out",
    double execution_bound = 1.0) {
    const std::vector<PhaseNominalSample> samples =
        zeroNominalSamples(execution_bound);
    std::map<std::string, std::string> metadata = artifactMetadata(evidence);
    metadata["recovery_artifact_hash"] =
        NominalSequenceArtifact::canonicalRecoveryArtifactHash(
            metadata, samples);
    NominalSequenceArtifact artifact;
    const NominalArtifactLoadResult result = artifact.assignValidated(
        metadata, samples, "<augmented-unit-test>");
    EXPECT_TRUE(result.success) << result.status << ": " << result.detail;
    return artifact;
}

ReferencePath shortReference() {
    std::vector<TrajectoryPoint> points(2);
    points[1].x = 0.09;
    ReferencePath reference;
    reference.setPoints(points, "map");
    EXPECT_FALSE(reference.empty());
    return reference;
}

PhaseRejoinRuntimeContract runtimeContract(double execution_bound = 1.0) {
    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    PhaseRejoinRuntimeContract runtime;
    runtime.dt = manifest::kDt;
    runtime.slosh_model = compiled.slosh;
    runtime.min_command_v = manifest::kLinearOutputMin;
    runtime.max_command_v = manifest::kLinearOutputMax;
    runtime.max_abs_command_omega = manifest::kAngularOutputMax;
    SloshDynamics slosh;
    runtime.liquid_model_configured = slosh.configure(compiled.slosh);
    const double omega_n = slosh.omegaN();
    runtime.two_zeta_omega_n =
        2.0 * compiled.slosh.damping_ratio * omega_n;
    runtime.omega_n_sq = omega_n * omega_n;
    runtime.kappa_x = 1.0;
    runtime.kappa_y = 1.0;
    runtime.delay_augmented_solver_requested = true;
    runtime.execution_contract_id = compiled.execution.contract_id;
    runtime.execution_contract_hash = compiled.execution.contract_hash;
    runtime.execution_state_width = compiled.state_width;
    runtime.linear_buffer_count = manifest::kLinearBufferCount;
    runtime.angular_buffer_count = manifest::kAngularBufferCount;
    runtime.solver_control_width = compiled.control_width;
    runtime.execution_front_steps = compiled.execution_front_steps;
    runtime.solver_horizon_steps = compiled.horizon_steps;
    runtime.max_published_acceleration = compiled.acceleration_max;
    runtime.max_published_angular_acceleration =
        compiled.angular_acceleration_max;
    runtime.parameter_schema_version = compiled.parameter_schema_version;
    runtime.parameter_schema_id = compiled.parameter_schema_id;
    runtime.parameter_schema_hash = compiled.parameter_schema_hash;
    runtime.recovery_artifact_hash = recoveryArtifactHash(
        "empirical_held_out", execution_bound);
    runtime.execution_compatibility_contract =
        compiled.execution_compatibility_contract;
    runtime.solver_capabilities = compiled.capabilities;
    runtime.required_solver_capabilities =
        kDelayAugmentedPhaseFormalCapabilities;
    runtime.delay_augmented_weights = costWeights();
    return runtime;
}

PhaseRejoinParams phaseParams() {
    PhaseRejoinParams params;
    params.mode = PhaseRejoinMode::Enforce;
    params.required_contract_id = kNominalContractId;
    params.required_frame_id = "map";
    params.liquid_horizon_steps = manifest::kLiquidHorizonSteps;
    params.max_residual_v = 0.08;
    params.max_residual_omega = 0.20;
    return params;
}

SolverParams solverParams(double execution_bound = 1.0) {
    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    SolverParams params;
    params.solver_backend = kSolverBackendDelayAugmentedPhaseAcados;
    params.v_max = manifest::kLinearOutputMax;
    params.omega_max = manifest::kAngularOutputMax;
    params.a_max = manifest::kAccelerationMax;
    params.alpha_max = manifest::kAngularAccelerationMax;
    params.slosh = compiled.slosh;
    params.terminal.enable = false;
    params.terminal.goal_tolerance = 0.001;
    params.terminal.slowdown_enable = false;
    params.terminal.capture_stop_enable = false;
    params.terminal.command_clamp_enable = false;
    DelayAugmentedPhaseBackendParams& augmented =
        params.delay_augmented_phase;
    augmented.enabled = true;
    augmented.execution_contract_id = compiled.execution.contract_id;
    augmented.execution_contract_hash = compiled.execution.contract_hash;
    augmented.expected_state_width = compiled.state_width;
    augmented.expected_control_width = compiled.control_width;
    augmented.expected_horizon_steps = compiled.horizon_steps;
    augmented.parameter_schema_version = compiled.parameter_schema_version;
    augmented.parameter_schema_id = compiled.parameter_schema_id;
    augmented.parameter_schema_hash = compiled.parameter_schema_hash;
    augmented.expected_recovery_artifact_hash = recoveryArtifactHash(
        "empirical_held_out", execution_bound);
    augmented.required_capabilities =
        kDelayAugmentedPhaseFormalCapabilities;
    return params;
}

VariantConfig augmentedVariant() {
    VariantConfig variant;
    variant.name = "B_slosh";
    variant.slosh_enable = true;
    variant.w_contour = 1.0;
    variant.w_lag = 0.2;
    variant.w_progress = 0.2;
    variant.w_v = 1.0;
    variant.w_control = 0.1;
    variant.w_accel = 0.0;
    variant.w_alpha = 0.1;
    variant.w_vs = 0.3;
    variant.w_slosh = 1.0;
    return variant;
}

void pushZero(CommandHistoryBuffer& history, double stamp_sec) {
    TimedCommandSample sample;
    sample.stamp_ns = secondsToNanoseconds(stamp_sec);
    history.push(sample);
}

PublishEpochEstimate publishEstimate() {
    PublishLatencyModel model;
    PublishLatencyModelConfig config;
    config.enabled = true;
    config.estimated_dc_sec = 0.01;
    std::string error;
    EXPECT_TRUE(model.configure(config, error)) << error;
    CycleTimingContract cycle;
    cycle.cycle_id = 42;
    cycle.cycle_start_stamp_ns = secondsToNanoseconds(9.95);
    cycle.control_period_sec = manifest::kDt;
    return model.estimate(cycle);
}

ExecutionHorizonContext contextFromFinalHistory(
    CommandHistoryBuffer& history) {
    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    ExecutionHorizonContextBuilder builder;
    ExecutionHorizonBuilderConfig config;
    config.command_timeout_sec = 0.5;
    config.max_alignment_sec = 0.5;
    config.max_integration_step_sec = 0.01;
    config.min_integration_step_sec = 0.0001;
    std::string error;
    EXPECT_TRUE(builder.configure(
        compiled.execution, compiled.slosh, config, error)) << error;
    ExecutionHorizonBuildRequest request;
    request.source_epoch_ns = secondsToNanoseconds(9.90);
    request.publish_epoch_estimate = publishEstimate();
    request.command_history = &history;
    request.expected_execution_contract_hash =
        compiled.execution.contract_hash;
    request.initial_progress_s = 0.0;
    request.liquid_horizon_steps = manifest::kLiquidHorizonSteps;
    const ExecutionHorizonBuildResult result = builder.build(request);
    EXPECT_TRUE(result.valid) << result.status;
    EXPECT_TRUE(result.alignment.history_complete);
    return result.context;
}

class RecordingSink : public ICommandSink {
public:
    StampNs publicationTimeNs() override { return now_ns; }

    PublicationReceipt publish(const FinalCommand& final) override {
        last = final;
        PublicationReceipt receipt;
        receipt.cycle_id = final.cycle_id;
        receipt.attempted = true;
        receipt.delivered = final.publish_enabled;
        receipt.command = final.command;
        receipt.actual_publish_stamp_ns = final.publish_enabled ? now_ns : 0;
        receipt.status = final.publish_enabled ? "RECORDED" : "DISABLED";
        return receipt;
    }

    StampNs now_ns = secondsToNanoseconds(9.96);
    FinalCommand last;
};

class CountingSolverSession : public SolverSession {
public:
    explicit CountingSolverSession(SolverSession& delegate)
        : delegate_(delegate) {}

    bool solve(const SolverInput& input, SolverOutput& output) override {
        ++calls;
        return delegate_.solve(input, output);
    }

    int calls = 0;

private:
    SolverSession& delegate_;
};

struct OnlineCycleFixture {
    explicit OnlineCycleFixture(double execution_bound = 1.0)
        : solver_session(problem), engine(solver_session) {
        for (double stamp :
             {9.60, 9.65, 9.70, 9.75, 9.80, 9.85, 9.90, 9.95}) {
            pushZero(history, stamp);
        }
        history.configure(2.0);
        const SolverConfigureResult configured = problem.configure(
            solverParams(execution_bound), augmentedVariant());
        EXPECT_TRUE(configured.success)
            << configured.status << ": " << configured.detail;
        reference = shortReference();
        problem.setReferencePath(reference);

        std::string error;
        EXPECT_TRUE(engine.configurePhaseRejoin(phaseParams(), error))
            << error;
        artifact = augmentedArtifact("empirical_held_out", execution_bound);
        artifact_path = "/tmp/spmpc_augmented_online_" +
            std::to_string(static_cast<long long>(::getpid())) + ".csv";
        const NominalArtifactLoadResult write =
            artifact.writeCanonicalCsv(artifact_path, true);
        EXPECT_TRUE(write.success) << write.status << ": " << write.detail;
        const NominalArtifactLoadResult load =
            engine.loadPhaseRejoinArtifact(artifact_path);
        EXPECT_TRUE(load.success) << load.status << ": " << load.detail;
        EXPECT_TRUE(engine.validatePhaseRejoinRuntimeContract(
            runtimeContract(execution_bound), reference, error)) << error;

        SafetySupervisorConfig safety;
        safety.terminal_spin.enable = false;
        safety.tracking.enable = false;
        EXPECT_TRUE(engine.configureSafety(safety, error)) << error;
        CommandPipelineConfig pipeline;
        pipeline.control_frequency = 1.0 / manifest::kDt;
        pipeline.linear_accel_limit_enable = false;
        pipeline.angular_limit_enable = false;
        EXPECT_TRUE(engine.configureCommandPipeline(pipeline, error))
            << error;
        PublishLatencyModelConfig latency;
        latency.enabled = true;
        latency.estimated_dc_sec = 0.01;
        EXPECT_TRUE(engine.configurePublishLatency(latency, error)) << error;
        execution_context = contextFromFinalHistory(history);
    }

    ~OnlineCycleFixture() {
        if (!artifact_path.empty()) std::remove(artifact_path.c_str());
    }

    ControlCycleRequest request() {
        ControlCycleRequest request;
        request.cycle_id = 42;
        request.cycle_start_ns = secondsToNanoseconds(9.95);
        request.control_period_sec = manifest::kDt;
        request.period_sec = manifest::kDt;
        request.publish_epoch_estimate = publishEstimate();
        request.solver_input.dt = manifest::kDt;
        request.solver_input.horizon_steps = manifest::kHorizonSteps;
        request.solver_input.execution_horizon = execution_context;
        request.solver_origin_at_execution_front = false;
        request.execution_front_steps = manifest::kExecutionFrontSteps;
        request.publish_enabled = true;
        request.command_sink = &sink;
        request.command_history = &history;
        return request;
    }

    SpmpcProblem problem;
    CountingSolverSession solver_session;
    ControlCycleEngine engine;
    ReferencePath reference;
    NominalSequenceArtifact artifact;
    std::string artifact_path;
    CommandHistoryBuffer history;
    RecordingSink sink;
    ExecutionHorizonContext execution_context;
};

double percentile(const std::vector<double>& sorted, double probability) {
    if (sorted.empty()) return 0.0;
    const std::size_t index = std::min(
        sorted.size() - 1,
        static_cast<std::size_t>(
            std::ceil(probability * sorted.size())) - 1);
    return sorted[index];
}

}  // namespace

TEST(DelayAugmentedPhaseOnline,
     ExplicitCycleUses22DBackendAndPublishesOneAuditedCommandTruth) {
    if (!DelayAugmentedPhaseAcadosSolver::compiled()) {
        GTEST_SKIP() << "delay-augmented generated capsule is unavailable";
    }
    OnlineCycleFixture fixture;
    const ControlCycleResult result = fixture.engine.step(fixture.request());

    ASSERT_TRUE(result.solve_returned);
    ASSERT_TRUE(result.solver_success) << result.solver_output.status;
    EXPECT_EQ(result.solver_output.pre_solve_snapshot.backend,
              kSolverBackendDelayAugmentedPhaseAcados);
    EXPECT_EQ(result.solver_output.pre_solve_snapshot.state_width, 22);
    EXPECT_EQ(result.solver_output.pre_solve_snapshot.control_width, 3);
    EXPECT_EQ(result.solver_output.pre_solve_snapshot.parameter_width,
              manifest::kParameterCount);
    EXPECT_EQ(result.solver_output.predicted_horizon.backend,
              kSolverBackendDelayAugmentedPhaseAcados);
    EXPECT_EQ(result.solver_output.predicted_horizon.states.size(), 11u);
    ASSERT_FALSE(result.solver_output.predicted_horizon.states.empty());
    EXPECT_DOUBLE_EQ(
        result.solver_output.progress_abs_s,
        result.solver_output.predicted_horizon.states.front().s);
    EXPECT_TRUE(result.phase_decision.current_execution_compatible);
    EXPECT_TRUE(result.phase_decision.terminal_execution_compatible);
    EXPECT_TRUE(result.phase_decision.terminal_gate_accepted);
    EXPECT_TRUE(result.phase_committed);
    EXPECT_TRUE(result.publication.history_committed);
    EXPECT_TRUE(result.telemetry.command_history_committed);
    EXPECT_EQ(result.telemetry.cycle_id, 42u);
    EXPECT_EQ(fixture.sink.last.cycle_id, 42u);
    EXPECT_DOUBLE_EQ(result.final_command.linear,
                     fixture.sink.last.command.linear);
    EXPECT_DOUBLE_EQ(result.final_command.angular,
                     fixture.sink.last.command.angular);
    TimedCommandSample committed;
    ASSERT_TRUE(fixture.history.sampleAt(
        fixture.sink.now_ns, committed));
    EXPECT_EQ(committed.stamp_ns, fixture.sink.now_ns);
    EXPECT_DOUBLE_EQ(committed.command.linear, result.final_command.linear);
    EXPECT_DOUBLE_EQ(committed.command.angular, result.final_command.angular);
}

TEST(DelayAugmentedPhaseOnline,
     NoCompatibleCandidateFailsClosedBeforeSolverInvocation) {
    OnlineCycleFixture fixture(0.01);
    ControlCycleRequest request = fixture.request();
    request.solver_input.execution_horizon.initial_state.linear
        .pending_commands.assign(
            static_cast<std::size_t>(manifest::kLinearBufferCount), 0.02);

    const ControlCycleResult result = fixture.engine.step(request);

    EXPECT_EQ(fixture.solver_session.calls, 0);
    EXPECT_FALSE(result.solve_returned);
    EXPECT_FALSE(result.solver_success);
    EXPECT_FALSE(result.telemetry.solve_attempted);
    EXPECT_EQ(result.phase_preparation.status,
              "NO_EXECUTION_COMPATIBLE_CANDIDATE");
    EXPECT_EQ(result.solver_output.status,
              "NOT_RUN_NO_EXECUTION_COMPATIBLE_CANDIDATE");
    EXPECT_EQ(result.phase_decision.status,
              "ENFORCE_NOT_READY_STOP_NO_EXECUTION_COMPATIBLE_CANDIDATE");
    EXPECT_EQ(result.output.status, result.phase_decision.status);
    EXPECT_EQ(result.telemetry.solver_status, result.solver_output.status);
    EXPECT_EQ(result.telemetry.status, result.output.status);
    EXPECT_TRUE(result.phase_decision.controlled_stop_used);
    EXPECT_EQ(result.decision.source, CommandSource::PhaseRejoin);
    EXPECT_FALSE(result.decision.accepted);
    EXPECT_DOUBLE_EQ(result.final_command.linear, 0.0);
    EXPECT_DOUBLE_EQ(result.final_command.angular, 0.0);
}

TEST(DelayAugmentedPhaseOnline,
     NoHorizonCompatibleCandidateFailsClosedBeforeSolverInvocation) {
    OnlineCycleFixture fixture;
    ControlCycleRequest request = fixture.request();
    // This tail is inside the current B_exec, but cannot reach the nominal
    // published-command interval within one frozen rate step.  The complete
    // causal horizon filter must therefore reject it before the OCP runs.
    request.solver_input.execution_horizon.initial_state.linear
        .pending_commands.back() = 0.5;

    const ControlCycleResult result = fixture.engine.step(request);

    EXPECT_EQ(fixture.solver_session.calls, 0);
    EXPECT_FALSE(result.solve_returned);
    EXPECT_FALSE(result.solver_success);
    EXPECT_FALSE(result.telemetry.solve_attempted);
    EXPECT_EQ(result.phase_preparation.status,
              "NO_EXECUTION_HORIZON_COMPATIBLE_CANDIDATE");
    EXPECT_EQ(result.solver_output.status,
              "NOT_RUN_NO_EXECUTION_HORIZON_COMPATIBLE_CANDIDATE");
    EXPECT_EQ(
        result.phase_decision.status,
        "ENFORCE_NOT_READY_STOP_"
        "NO_EXECUTION_HORIZON_COMPATIBLE_CANDIDATE");
    EXPECT_EQ(result.telemetry.solver_status, result.solver_output.status);
    EXPECT_TRUE(result.phase_decision.controlled_stop_used);
    EXPECT_EQ(result.decision.source, CommandSource::PhaseRejoin);
    EXPECT_FALSE(result.decision.accepted);
    EXPECT_DOUBLE_EQ(result.final_command.linear, 0.0);
    EXPECT_DOUBLE_EQ(result.final_command.angular, 0.0);
}

TEST(DelayAugmentedPhaseOnline,
     ParameterImageMatchesManifestAndMutationsFailClosed) {
    if (!DelayAugmentedPhaseAcadosSolver::compiled()) {
        GTEST_SKIP() << "delay-augmented generated capsule is unavailable";
    }
    OnlineCycleFixture fixture;
    const ControlCycleResult cycle = fixture.engine.step(fixture.request());
    ASSERT_TRUE(cycle.solver_success) << cycle.solver_output.status;
    const DelayAugmentedPhaseSolverContext& context =
        cycle.phase_preparation.solver_context.delay_augmented;
    const DelayAugmentedPhaseParameterMatrix image =
        DelayAugmentedPhaseParameterBuilder::build(context);
    ASSERT_TRUE(image.valid) << image.status;
    ASSERT_EQ(image.stage_count, manifest::kHorizonSteps + 1);
    ASSERT_EQ(image.parameter_width, manifest::kParameterCount);
    for (int index = 0; index < manifest::kParameterCount; ++index) {
        EXPECT_EQ(image.parameter_names[static_cast<std::size_t>(index)],
                  manifest::kParameterNames[index]);
    }
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        EXPECT_DOUBLE_EQ(image.value(stage, manifest::kNominalStateOffset),
                         0.09);
        EXPECT_DOUBLE_EQ(
            image.value(stage, manifest::kNominalControlOffset), 0.0);
        EXPECT_DOUBLE_EQ(
            image.value(stage, manifest::kNominalPublishOffset), 0.0);
        EXPECT_DOUBLE_EQ(
            image.value(stage, manifest::kResidualBoundOffset), 0.08);
        EXPECT_DOUBLE_EQ(image.value(stage, manifest::kWeightOffset), 1.0);
        EXPECT_DOUBLE_EQ(
            image.value(stage, manifest::kGateRadiusOffset), 1.0);
        EXPECT_DOUBLE_EQ(
            image.value(stage, manifest::kExecutionBoundOffset), 1.0);
    }

    DelayAugmentedPhaseSolverContext mutated = context;
    mutated.parameter_schema_hash[0] =
        mutated.parameter_schema_hash[0] == '0' ? '1' : '0';
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.state_width = 21;
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.stages.pop_back();
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.terminal_empirical_gate_bound = false;
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.max_residual_v =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.max_residual_v =
        manifest::kLinearOutputMax - manifest::kLinearOutputMin + 0.01;
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.stages.front().augmented_execution.linear.actuator_output =
        manifest::kLinearOutputMax + 0.01;
    mutated.stages.front().augmented_execution.robot.v =
        manifest::kLinearOutputMax + 0.01;
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.stages.front().augmented_execution.angular.pending_commands[0] =
        manifest::kAngularOutputMax + 0.01;
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.stages.front().a = manifest::kAccelerationMax + 0.01;
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.stages.front().radii.x = 1.0e-12;
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);
    mutated = context;
    mutated.stages.front().execution_bounds.linear_actuator_output = 1.0e-12;
    EXPECT_FALSE(DelayAugmentedPhaseParameterBuilder::build(mutated).valid);

    DelayAugmentedPhaseParameterMatrix malformed = image;
    malformed.values.pop_back();
    EXPECT_FALSE(malformed.hasCanonicalShape());
    EXPECT_EQ(malformed.stageData(0), nullptr);
    malformed = image;
    std::swap(malformed.parameter_names[0], malformed.parameter_names[1]);
    EXPECT_FALSE(malformed.hasCanonicalShape());
    EXPECT_EQ(malformed.stageData(0), nullptr);

    DelayAugmentedPhaseOnlineSolver configured_solver;
    ASSERT_TRUE(configured_solver.configure(
        solverParams(), augmentedVariant()).success);
    SolverInput mismatched_cost_input = cycle.solver_input;
    mismatched_cost_input.phase_rejoin =
        cycle.phase_preparation.solver_context;
    mismatched_cost_input.phase_rejoin.delay_augmented.weights.position += 1.0;
    SolverOutput mismatched_cost_output;
    EXPECT_FALSE(configured_solver.solve(
        mismatched_cost_input, fixture.reference, mismatched_cost_output));
    EXPECT_EQ(mismatched_cost_output.status,
              "DELAY_AUGMENTED_COST_CONTRACT_MISMATCH");

    DelayAugmentedPhaseOnlineSolver solver;
    SolverParams params = solverParams();
    params.delay_augmented_phase.expected_state_width = 21;
    EXPECT_FALSE(solver.configure(params, augmentedVariant()).success);
    params = solverParams();
    params.delay_augmented_phase.expected_recovery_artifact_hash.clear();
    EXPECT_EQ(
        solver.configure(params, augmentedVariant()).status,
        "DELAY_AUGMENTED_RECOVERY_ASSET_NOT_FROZEN");

    VariantConfig negative_raw_acceleration = augmentedVariant();
    negative_raw_acceleration.w_control = 0.1;
    negative_raw_acceleration.w_accel = -0.05;
    EXPECT_EQ(
        solver.configure(solverParams(), negative_raw_acceleration).status,
        "DELAY_AUGMENTED_RUNTIME_MODEL_MISMATCH");
}

TEST(DelayAugmentedPhaseOnline,
     FormalAssetAndCurrentTerminalCompatibilityCannotBeBypassed) {
    PhaseRejoinCoordinator coordinator;
    std::string error;
    ASSERT_TRUE(coordinator.configure(phaseParams(), error)) << error;
    ASSERT_TRUE(coordinator.setArtifact(augmentedArtifact(), error)) << error;
    ASSERT_TRUE(coordinator.validateRuntimeContract(
        runtimeContract(), shortReference(), error)) << error;

    ExecutionAugmentedState current = zeroExecution(0);
    // Keep this recovery-path contract test at the nominal robot pose.  A
    // 9 cm tracking error now correctly produces a bounded feedback action,
    // but that action exceeds the one-cycle rate limit from a zero command and
    // belongs in the dedicated rate-rejection assertion below.
    current.robot.x = 0.09;
    const PhaseRejoinPreparation duplicate_front_shift = coordinator.prepare(
        current.robot, current.slosh,
        manifest::kExecutionFrontSteps, manifest::kHorizonSteps,
        10.0, true, true);
    EXPECT_FALSE(duplicate_front_shift.ready);
    EXPECT_EQ(duplicate_front_shift.status,
              "DELAY_AUGMENTED_ORIGIN_CONTRACT_MISMATCH");

    const ExecutionHorizonContext current_horizon =
        horizonFromExecution(current);
    const auto preparation = coordinator.prepare(
        current.robot, current.slosh,
        manifest::kExecutionFrontSteps, manifest::kHorizonSteps,
        10.0, false, true, &current, &current_horizon);
    ASSERT_TRUE(preparation.ready) << preparation.status;
    PhaseSolveView solve;
    solve.current_execution_state_available = true;
    solve.current_execution = current;
    solve.terminal_execution_state_available = true;
    solve.terminal_execution = zeroExecution(manifest::kHorizonSteps);
    solve.terminal_state_available = true;
    const PhaseRejoinDecision accepted = coordinator.decide(
        preparation, RobotState{}, SloshState{}, true, solve);
    EXPECT_EQ(accepted.status, "ENFORCE_TERMINAL_ACCEPTED");

    SolverOutput failed_output;
    failed_output.failure_kind = SolverFailureKind::Optimization;
    const PhaseSolveView failed_view = makePhaseSolveView(
        failed_output, preparation.solver_terminal_step, &current);
    const PhaseRejoinDecision failed_recovery = coordinator.decide(
        preparation, RobotState{}, SloshState{}, false, failed_view);
    EXPECT_TRUE(failed_recovery.current_execution_compatible);
    EXPECT_TRUE(failed_recovery.recovery_command_used);
    EXPECT_EQ(failed_recovery.status, "ENFORCE_SOLVER_FAILED_RECOVERY");

    SolverOutput integrity_failure;
    integrity_failure.failure_kind = SolverFailureKind::Integrity;
    const PhaseSolveView integrity_view = makePhaseSolveView(
        integrity_failure, preparation.solver_terminal_step, &current);
    const PhaseRejoinDecision integrity_stop = coordinator.decide(
        preparation, RobotState{}, SloshState{}, false, integrity_view);
    EXPECT_FALSE(integrity_stop.recovery_command_used);
    EXPECT_TRUE(integrity_stop.controlled_stop_used);
    EXPECT_EQ(integrity_stop.status,
              "ENFORCE_SOLVER_INTEGRITY_FAILURE_STOP");

    ExecutionAugmentedState rate_mismatched = current;
    rate_mismatched.linear.pending_commands.back() =
        manifest::kAccelerationMax * manifest::kDt + 0.01;
    const PhaseSolveView rate_mismatched_view = makePhaseSolveView(
        failed_output, preparation.solver_terminal_step,
        &rate_mismatched);
    const PhaseRejoinDecision rate_limited_recovery = coordinator.decide(
        preparation, RobotState{}, SloshState{}, false,
        rate_mismatched_view);
    EXPECT_TRUE(rate_limited_recovery.current_execution_compatible);
    EXPECT_TRUE(rate_limited_recovery.recovery_command_used);
    EXPECT_FALSE(rate_limited_recovery.controlled_stop_used);
    EXPECT_NEAR(
        rate_limited_recovery.output_cmd_v,
        rate_mismatched.linear.pending_commands.back() -
            manifest::kAccelerationMax * manifest::kDt,
        1.0e-12);
    EXPECT_EQ(
        rate_limited_recovery.status,
        "ENFORCE_SOLVER_FAILED_RECOVERY_RATE_LIMITED");

    solve.current_execution.linear.pending_commands.front() = 2.0;
    const PhaseRejoinDecision bad_current = coordinator.decide(
        preparation, RobotState{}, SloshState{}, true, solve);
    EXPECT_TRUE(bad_current.controlled_stop_used);
    EXPECT_FALSE(bad_current.command_contract_consistent);

    solve.current_execution = current;
    solve.terminal_execution.angular.pending_commands.back() = 2.0;
    const PhaseRejoinDecision bad_terminal_execution = coordinator.decide(
        preparation, RobotState{}, SloshState{}, true, solve);
    EXPECT_TRUE(bad_terminal_execution.recovery_command_used);
    EXPECT_FALSE(bad_terminal_execution.command_contract_consistent);

    solve.terminal_execution = zeroExecution(manifest::kHorizonSteps);
    solve.terminal_state_available = false;
    const PhaseRejoinDecision missing_terminal_9d = coordinator.decide(
        preparation, RobotState{}, SloshState{}, true, solve);
    EXPECT_TRUE(missing_terminal_9d.recovery_command_used);
    EXPECT_FALSE(missing_terminal_9d.command_contract_consistent);

    PhaseRejoinCoordinator development;
    PhaseRejoinParams development_params = phaseParams();
    development_params.allow_development_artifact_in_enforce = true;
    ASSERT_TRUE(development.configure(development_params, error)) << error;
    ASSERT_TRUE(development.setArtifact(
        augmentedArtifact("development_only"), error)) << error;
    EXPECT_FALSE(development.validateRuntimeContract(
        runtimeContract(), shortReference(), error));
    EXPECT_EQ(error, "FORMAL_RECOVERY_ASSET_REQUIRED");

    PhaseRejoinRuntimeContract wrong_recovery = runtimeContract();
    wrong_recovery.recovery_artifact_hash = std::string(64, 'b');
    EXPECT_FALSE(coordinator.validateRuntimeContract(
        wrong_recovery, shortReference(), error));
    EXPECT_EQ(error, "RECOVERY_ARTIFACT_HASH_MISMATCH");

    PhaseRejoinRuntimeContract wrong_rate = runtimeContract();
    wrong_rate.max_published_acceleration += 0.01;
    EXPECT_FALSE(coordinator.validateRuntimeContract(
        wrong_rate, shortReference(), error));
    EXPECT_EQ(error, "AUGMENTED_SOLVER_RATE_CONTRACT_MISMATCH");

    PhaseRejoinCoordinator oversized_residual;
    PhaseRejoinParams oversized_params = phaseParams();
    oversized_params.max_residual_v =
        manifest::kLinearOutputMax - manifest::kLinearOutputMin + 0.01;
    ASSERT_TRUE(oversized_residual.configure(oversized_params, error));
    ASSERT_TRUE(oversized_residual.setArtifact(
        augmentedArtifact(), error));
    EXPECT_FALSE(oversized_residual.validateRuntimeContract(
        runtimeContract(), shortReference(), error));
    EXPECT_EQ(error, "AUGMENTED_RESIDUAL_BOUND_MISMATCH");
}

TEST(DelayAugmentedPhaseOnline,
     GeneratedSolverEnforcesPublishedResidualBeforeCoordinator) {
    if (!DelayAugmentedPhaseAcadosSolver::compiled()) {
        GTEST_SKIP() << "delay-augmented generated capsule is unavailable";
    }
    OnlineCycleFixture fixture;
    const ControlCycleResult cycle = fixture.engine.step(fixture.request());
    ASSERT_TRUE(cycle.solver_success) << cycle.solver_output.status;

    SolverInput input = cycle.solver_input;
    input.phase_rejoin = cycle.phase_preparation.solver_context;
    input.execution_horizon.initial_state.linear.pending_commands.back() =
        0.02;
    input.phase_rejoin.delay_augmented.max_residual_v = 0.005;

    DelayAugmentedPhaseOnlineSolver solver;
    ASSERT_TRUE(solver.configure(solverParams(), augmentedVariant()).success);
    SolverOutput output;
    ASSERT_TRUE(solver.solve(input, fixture.reference, output))
        << output.status;
    EXPECT_TRUE(output.pre_solve_snapshot.warm_start_constraint_audit.evaluated);
    EXPECT_FALSE(output.pre_solve_snapshot.warm_start_constraint_audit.passed);
    EXPECT_GT(output.pre_solve_snapshot.warm_start_constraint_audit.max_violation,
              output.pre_solve_snapshot.warm_start_constraint_audit.tolerance);
    EXPECT_TRUE(output.pre_solve_snapshot.solution_constraint_audit.evaluated);
    EXPECT_TRUE(output.pre_solve_snapshot.solution_constraint_audit.passed);
    EXPECT_LE(std::abs(output.cmd_v), 0.005 + 1.0e-6);
    EXPECT_LE(std::abs(output.cmd_omega), 1.0e-6);
}

TEST(DelayAugmentedPhaseOnline,
     CompleteHistoryAndMonotonicTimeAreStrictAdmissionRequirements) {
    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    ExecutionHorizonContextBuilder builder;
    ExecutionHorizonBuilderConfig config;
    config.command_timeout_sec = 0.5;
    config.max_alignment_sec = 0.5;
    std::string error;
    ASSERT_TRUE(builder.configure(
        compiled.execution, compiled.slosh, config, error)) << error;

    CommandHistoryBuffer incomplete;
    incomplete.configure(2.0);
    pushZero(incomplete, 9.80);
    pushZero(incomplete, 9.95);
    ExecutionHorizonBuildRequest request;
    request.source_epoch_ns = secondsToNanoseconds(9.90);
    request.publish_epoch_estimate = publishEstimate();
    request.command_history = &incomplete;
    request.expected_execution_contract_hash =
        compiled.execution.contract_hash;
    request.liquid_horizon_steps = manifest::kLiquidHorizonSteps;
    EXPECT_FALSE(builder.build(request).valid);

    CommandHistoryBuffer regressed;
    regressed.configure(2.0);
    pushZero(regressed, 9.90);
    pushZero(regressed, 9.80);
    request.command_history = &regressed;
    const ExecutionHorizonBuildResult after_regression =
        builder.build(request);
    EXPECT_FALSE(after_regression.valid);
    EXPECT_NE(after_regression.status.find("HISTORY"), std::string::npos);

    PhaseRejoinCoordinator coordinator;
    ASSERT_TRUE(coordinator.configure(phaseParams(), error)) << error;
    ASSERT_TRUE(coordinator.setArtifact(augmentedArtifact(), error)) << error;
    ASSERT_TRUE(coordinator.validateRuntimeContract(
        runtimeContract(), shortReference(), error)) << error;
    const ExecutionAugmentedState current = zeroExecution(0);
    const ExecutionHorizonContext current_horizon =
        horizonFromExecution(current);
    ASSERT_TRUE(coordinator.prepare(
        current.robot, current.slosh,
        manifest::kExecutionFrontSteps, manifest::kHorizonSteps,
        10.0, false, true, &current, &current_horizon).ready);
    const PhaseRejoinPreparation regressed_phase = coordinator.prepare(
        current.robot, current.slosh,
        manifest::kExecutionFrontSteps, manifest::kHorizonSteps,
        9.0, false, true, &current, &current_horizon);
    EXPECT_FALSE(regressed_phase.ready);
    EXPECT_EQ(regressed_phase.status, "CLOCK_REGRESSION");
}

TEST(DelayAugmentedPhaseOnline,
     ThirtyHzWallSolveLatencyIsRecordedWithBackendEvidence) {
    if (!DelayAugmentedPhaseAcadosSolver::compiled()) {
        GTEST_SKIP() << "delay-augmented generated capsule is unavailable";
    }
    OnlineCycleFixture fixture;
    const ControlCycleResult cycle = fixture.engine.step(fixture.request());
    ASSERT_TRUE(cycle.solver_success) << cycle.solver_output.status;

    DelayAugmentedPhaseOnlineSolver solver;
    const SolverConfigureResult configured = solver.configure(
        solverParams(), augmentedVariant());
    ASSERT_TRUE(configured.success)
        << configured.status << ": " << configured.detail;
    SolverInput input = cycle.solver_input;
    input.phase_rejoin = cycle.phase_preparation.solver_context;

    for (int warmup = 0; warmup < 5; ++warmup) {
        SolverOutput output;
        ASSERT_TRUE(solver.solve(input, fixture.reference, output))
            << output.status;
    }
    std::vector<double> wall_ms;
    wall_ms.reserve(100);
    int deadline_misses = 0;
    for (int sample = 0; sample < 100; ++sample) {
        SolverOutput output;
        const auto begin = std::chrono::steady_clock::now();
        const bool ok = solver.solve(input, fixture.reference, output);
        const auto end = std::chrono::steady_clock::now();
        ASSERT_TRUE(ok) << output.status;
        ASSERT_EQ(output.pre_solve_snapshot.backend,
                  kSolverBackendDelayAugmentedPhaseAcados);
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(end - begin).count();
        wall_ms.push_back(elapsed_ms);
        if (elapsed_ms > 1000.0 / 30.0) ++deadline_misses;
    }
    std::sort(wall_ms.begin(), wall_ms.end());
    const double p50 = percentile(wall_ms, 0.50);
    const double p95 = percentile(wall_ms, 0.95);
    const double maximum = wall_ms.back();
    std::cout << "[PERF_30HZ] backend="
              << kSolverBackendDelayAugmentedPhaseAcados
              << " samples=" << wall_ms.size()
              << " wall_ms_p50=" << p50
              << " wall_ms_p95=" << p95
              << " wall_ms_max=" << maximum
              << " deadline_misses=" << deadline_misses << std::endl;
    EXPECT_EQ(deadline_misses, 0);
}

}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
