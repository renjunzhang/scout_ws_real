#include <gtest/gtest.h>

#include "spmpc_local_planner/controller/control_cycle_engine.h"
#include "spmpc_local_planner/controller/phase_solve_adapter.h"
#include "phase_rejoin_artifact_fixture.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>

namespace spmpc_local_planner {
namespace {

class FakeSolverSession : public SolverSession {
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

class FakeCommandSink : public ICommandSink {
public:
    StampNs publicationTimeNs() override {
        return now_ns;
    }

    PublicationReceipt publish(const FinalCommand& command) override {
        ++calls;
        last_command = command;
        PublicationReceipt receipt;
        receipt.cycle_id = command.cycle_id;
        receipt.attempted = true;
        receipt.command = command.command;
        if (command.publish_enabled && deliver) {
            receipt.delivered = true;
            receipt.actual_publish_stamp_ns = now_ns;
            receipt.status = "FAKE_DELIVERED";
        } else {
            receipt.status = command.publish_enabled
                ? "FAKE_FAILED"
                : "PUBLISH_DISABLED";
        }
        return receipt;
    }

    StampNs now_ns = secondsToNanoseconds(1.0);
    bool deliver = true;
    int calls = 0;
    FinalCommand last_command;
};

struct EngineFixture {
    EngineFixture() : engine(solver) {
        PhaseRejoinParams phase;
        phase.mode = PhaseRejoinMode::Off;
        EXPECT_TRUE(engine.configurePhaseRejoin(phase, error)) << error;
        EXPECT_TRUE(engine.configureSafety(safety, error)) << error;
        CommandPipelineConfig pipeline;
        pipeline.linear_accel_limit_enable = false;
        pipeline.angular_limit_enable = false;
        EXPECT_TRUE(engine.configureCommandPipeline(pipeline, error))
            << error;
        history.configure(2.0);
    }

    ControlCycleRequest request() {
        ControlCycleRequest request;
        request.cycle_id = 1;
        request.cycle_start_ns = secondsToNanoseconds(0.9);
        request.solver_input.dt = 0.1;
        request.solver_input.horizon_steps = 10;
        request.period_sec = 0.1;
        request.control_period_sec = 0.1;
        request.command_sink = &sink;
        request.command_history = &history;
        return request;
    }

    FakeSolverSession solver;
    FakeCommandSink sink;
    CommandHistoryBuffer history;
    ControlCycleEngine engine;
    SafetySupervisorConfig safety;
    std::string error;
};

std::string goldenPath() {
    std::string source = __FILE__;
    const std::string marker = "/test/test_control_cycle_engine.cpp";
    const auto offset = source.rfind(marker);
    EXPECT_NE(offset, std::string::npos);
    return source.substr(0, offset) +
        "/test/golden/control_cycle_engine.csv";
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::string makePhaseArtifactFile() {
    const std::string path = "/tmp/spmpc_engine_phase_" +
        std::to_string(static_cast<long long>(::getpid())) + ".csv";
    std::ofstream out(path);
    out << spmpc_local_planner_test::completeArtifactText();
    out.close();
    return path;
}

ReferencePath phaseFixtureReference() {
    std::vector<TrajectoryPoint> points(2);
    points[1].x = 3.0;
    ReferencePath reference;
    reference.setPoints(points, "map");
    return reference;
}

PhaseRejoinRuntimeContract phaseFixtureRuntime() {
    PhaseRejoinRuntimeContract runtime;
    runtime.liquid_model_configured = true;
    runtime.dt = 0.1;
    runtime.two_zeta_omega_n = 0.2;
    runtime.omega_n_sq = 4.0;
    runtime.kappa_x = 1.0;
    runtime.kappa_y = 1.0;
    runtime.min_command_v = 0.0;
    runtime.max_command_v = 2.0;
    runtime.max_abs_command_omega = 3.0;
    return runtime;
}

void configureMonitorPhase(EngineFixture& fixture) {
    PhaseRejoinParams phase;
    phase.mode = PhaseRejoinMode::Monitor;
    phase.required_contract_id = "test_contract";
    ASSERT_TRUE(fixture.engine.configurePhaseRejoin(phase, fixture.error))
        << fixture.error;
    const std::string artifact_path = makePhaseArtifactFile();
    const NominalArtifactLoadResult loaded =
        fixture.engine.loadPhaseRejoinArtifact(artifact_path);
    std::remove(artifact_path.c_str());
    ASSERT_TRUE(loaded.success) << loaded.status << ": " << loaded.detail;
    ASSERT_TRUE(fixture.engine.validatePhaseRejoinRuntimeContract(
        phaseFixtureRuntime(), phaseFixtureReference(), fixture.error))
        << fixture.error;
}

void configureAcceptedMonitorSolve(EngineFixture& fixture) {
    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "OK";
    fixture.solver.next_output.cmd_v = 0.4;
    fixture.solver.next_output.cmd_omega = 0.0;
    fixture.solver.next_output.predicted_horizon.valid = true;
    fixture.solver.next_output.predicted_horizon.states.resize(4);
    fixture.solver.next_output.predicted_horizon.states[3].v = 1.0;
}

ControlCycleRequest monitorRequest(EngineFixture& fixture) {
    ControlCycleRequest request = fixture.request();
    request.prediction_valid = true;
    request.prediction_status = "OK";
    request.execution_front_robot.v = 1.0;
    request.solver_origin_at_execution_front = true;
    request.execution_front_steps = 0;
    request.phase_time_sec = 1.0;
    return request;
}

TEST(PhaseSolveAdapterTest, PreservesCommandAndRejectsUnavailableTerminal) {
    SolverOutput output;
    output.cmd_v = 0.41;
    output.cmd_omega = -0.27;
    output.predicted_horizon.valid = true;
    output.predicted_horizon.states.resize(1);

    PhaseSolveView view = makePhaseSolveView(output, -1);
    EXPECT_DOUBLE_EQ(0.41, view.cmd_v);
    EXPECT_DOUBLE_EQ(-0.27, view.cmd_omega);
    EXPECT_FALSE(view.terminal_state_available);

    view = makePhaseSolveView(output, 1);
    EXPECT_FALSE(view.terminal_state_available);

    output.predicted_horizon.valid = false;
    view = makePhaseSolveView(output, 0);
    EXPECT_FALSE(view.terminal_state_available);
}

TEST(PhaseSolveAdapterTest, MapsCompleteTerminalDomainState) {
    SolverOutput output;
    output.predicted_horizon.valid = true;
    output.predicted_horizon.states.resize(2);
    HorizonStateDebug& terminal = output.predicted_horizon.states[1];
    terminal.x = 1.1;
    terminal.y = -2.2;
    terminal.yaw = 3.3;
    terminal.v = -4.4;
    terminal.omega = 5.5;
    terminal.eta_x = -6.6;
    terminal.eta_x_dot = 7.7;
    terminal.eta_y = -8.8;
    terminal.eta_y_dot = 9.9;

    const PhaseSolveView view = makePhaseSolveView(output, 1);
    ASSERT_TRUE(view.terminal_state_available);
    EXPECT_DOUBLE_EQ(1.1, view.terminal_robot.x);
    EXPECT_DOUBLE_EQ(-2.2, view.terminal_robot.y);
    EXPECT_DOUBLE_EQ(3.3, view.terminal_robot.yaw);
    EXPECT_DOUBLE_EQ(-4.4, view.terminal_robot.v);
    EXPECT_DOUBLE_EQ(5.5, view.terminal_robot.omega);
    EXPECT_DOUBLE_EQ(-6.6, view.terminal_slosh.eta_x);
    EXPECT_DOUBLE_EQ(7.7, view.terminal_slosh.eta_x_dot);
    EXPECT_DOUBLE_EQ(-8.8, view.terminal_slosh.eta_y);
    EXPECT_DOUBLE_EQ(9.9, view.terminal_slosh.eta_y_dot);
}

TEST(ControlCycleEngineTest, InvokesInjectedSolverAndReturnsSolverCommand) {
    EngineFixture fixture;
    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "OK";
    fixture.solver.next_output.cmd_v = 0.4;
    fixture.solver.next_output.cmd_omega = -0.1;

    const ControlCycleResult result = fixture.engine.step(fixture.request());
    EXPECT_EQ(1, fixture.solver.calls);
    EXPECT_TRUE(result.output.success);
    EXPECT_EQ(CommandSource::Solver, result.decision.source);
    EXPECT_DOUBLE_EQ(0.4, result.decision.command.linear);
    EXPECT_DOUBLE_EQ(-0.1, result.decision.command.angular);
}

TEST(ControlCycleEngineTest, ProducesRosIndependentDecisionTelemetry) {
    EngineFixture fixture;
    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "OK";
    fixture.solver.next_output.cmd_v = 0.4;
    fixture.solver.next_output.cmd_omega = -0.1;
    fixture.solver.next_output.first_shot_debug.u0_a = 0.25;
    fixture.solver.next_output.first_shot_debug.u0_alpha = -0.5;
    ControlCycleRequest request = fixture.request();
    request.cycle_id = 42;
    request.cycle_start_ns = 123456789;
    request.solver_input.robot.v = 0.6;
    request.solver_input.robot.omega = -0.2;

    const ControlCycleResult result = fixture.engine.step(request);
    const ControlCycleTelemetrySnapshot& telemetry = result.telemetry;
    EXPECT_EQ(42u, telemetry.cycle_id);
    EXPECT_EQ(123456789, telemetry.cycle_start_ns);
    EXPECT_EQ("OK", telemetry.status);
    EXPECT_EQ("OK", telemetry.solver_status);
    EXPECT_EQ(CommandSource::Solver, telemetry.command_source);
    EXPECT_EQ("OK", telemetry.command_reason);
    EXPECT_TRUE(telemetry.solve_attempted);
    EXPECT_TRUE(telemetry.solve_returned);
    EXPECT_TRUE(telemetry.solve_success);
    EXPECT_TRUE(telemetry.command_accepted);
    EXPECT_DOUBLE_EQ(0.25, telemetry.solver_u0_a);
    EXPECT_DOUBLE_EQ(-0.5, telemetry.solver_u0_alpha);
    EXPECT_DOUBLE_EQ(-0.12, telemetry.planned_ay);
    EXPECT_DOUBLE_EQ(0.4, telemetry.final_command.linear);
    EXPECT_DOUBLE_EQ(-0.1, telemetry.final_command.angular);
    EXPECT_TRUE(telemetry.publication_attempted);
    EXPECT_TRUE(telemetry.command_was_published);
    EXPECT_TRUE(telemetry.publication_receipt_consistent);
    EXPECT_TRUE(telemetry.command_history_committed);
    EXPECT_DOUBLE_EQ(0.4, telemetry.published_command.linear);

    const CommandInterventionDebug intervention =
        makeCommandInterventionDebug(telemetry);
    EXPECT_DOUBLE_EQ(0.4, intervention.solver_cmd_v);
    EXPECT_DOUBLE_EQ(-0.1, intervention.post_gate_cmd_omega);
    EXPECT_TRUE(intervention.output_success);
    EXPECT_FALSE(intervention.zero_due_to_solver_failure);

    ControlCycleAuditDebug audit;
    applyControlCycleTelemetry(telemetry, audit);
    EXPECT_EQ(42u, audit.timing.cycle_id);
    EXPECT_EQ("OK", audit.status);
    EXPECT_TRUE(audit.solve_success);
    EXPECT_DOUBLE_EQ(0.25, audit.planned_ax);
    EXPECT_DOUBLE_EQ(0.4, audit.post_gate_cmd_v);
    EXPECT_TRUE(audit.command_was_published);
    EXPECT_DOUBLE_EQ(0.4, audit.published_cmd_v);
}

TEST(ControlCycleEngineTest, CommitsSolverProgressForNextSpeedReferenceCycle) {
    EngineFixture fixture;
    const std::string source = __FILE__;
    const std::string marker = "/test/test_control_cycle_engine.cpp";
    const auto offset = source.rfind(marker);
    ASSERT_NE(offset, std::string::npos);

    SpeedReferenceControllerConfig speed;
    speed.profile_enable = true;
    speed.profile_path = source.substr(0, offset) +
        "/test/fixtures/speed_profile.csv";
    speed.variant_v_ref = 0.25;
    speed.slosh_governor.enable = false;
    ASSERT_TRUE(
        fixture.engine.configureSpeedReference(speed).profile_load.success);

    SolverInput first_input;
    SpeedReferenceEvaluation first =
        fixture.engine.prepareSpeedReference(first_input);
    EXPECT_TRUE(first.applied);
    EXPECT_DOUBLE_EQ(first_input.v_ref_current, 0.10);

    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "OK";
    fixture.solver.next_output.progress_abs_s = 1.0;
    fixture.engine.step(fixture.request());

    SolverInput second_input;
    fixture.engine.prepareSpeedReference(second_input);
    EXPECT_DOUBLE_EQ(second_input.v_ref_current, 0.30);

    fixture.engine.resetForReference();
    SolverInput reset_input;
    fixture.engine.prepareSpeedReference(reset_input);
    EXPECT_DOUBLE_EQ(reset_input.v_ref_current, 0.10);
}

TEST(ControlCycleEngineTest, SolverFailureProducesFailClosedDecision) {
    EngineFixture fixture;
    fixture.solver.solve_return = false;
    fixture.solver.next_output.success = false;
    fixture.solver.next_output.status = "SOLVER_FAILED";

    const ControlCycleResult result = fixture.engine.step(fixture.request());
    EXPECT_FALSE(result.output.success);
    EXPECT_EQ(CommandSource::FailClosed, result.decision.source);
    EXPECT_DOUBLE_EQ(0.0, result.decision.command.linear);
    EXPECT_EQ("SOLVER_FAILED", result.output.status);
}

TEST(ControlCycleEngineTest, OwnsContiguousShiftedPlanAuditState) {
    EngineFixture fixture;
    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "OK";
    fixture.solver.next_output.first_shot_debug.u0_a = 0.4;
    fixture.solver.next_output.first_shot_debug.u0_alpha = -0.2;
    fixture.solver.next_output.predicted_horizon.valid = true;
    fixture.solver.next_output.predicted_horizon.controls.resize(2);
    fixture.solver.next_output.predicted_horizon.controls[1].a = 0.3;
    fixture.solver.next_output.predicted_horizon.controls[1].alpha_or_omega =
        -0.1;
    ControlCycleRequest request = fixture.request();
    request.cycle_id = 10;
    ControlCycleResult result = fixture.engine.step(request);
    EXPECT_FALSE(result.telemetry.previous_shifted_plan_available);

    fixture.solver.next_output.first_shot_debug.u0_a = 0.5;
    fixture.solver.next_output.first_shot_debug.u0_alpha = 0.2;
    request.cycle_id = 11;
    result = fixture.engine.step(request);
    EXPECT_TRUE(result.telemetry.previous_shifted_plan_available);
    EXPECT_EQ(result.telemetry.previous_plan_cycle_id, 10u);
    EXPECT_DOUBLE_EQ(result.telemetry.previous_shifted_plan_a, 0.3);
    EXPECT_DOUBLE_EQ(result.telemetry.previous_shifted_plan_alpha, -0.1);
    EXPECT_DOUBLE_EQ(result.telemetry.replanned_minus_shifted_a, 0.2);
    EXPECT_DOUBLE_EQ(result.telemetry.replanned_minus_shifted_alpha, 0.3);

    request.cycle_id = 13;
    result = fixture.engine.step(request);
    EXPECT_FALSE(result.telemetry.previous_shifted_plan_available);

    fixture.engine.resetForReference();
    request.cycle_id = 14;
    result = fixture.engine.step(request);
    EXPECT_FALSE(result.telemetry.previous_shifted_plan_available);
}

TEST(ControlCycleEngineTest, OwnsFinalLimiterAndExecutionContractStage) {
    EngineFixture fixture;
    CommandPipelineConfig pipeline;
    pipeline.control_frequency = 10.0;
    pipeline.linear_accel_limit_enable = true;
    pipeline.linear_accel_max = 1.0;
    pipeline.linear_accel_max_dt = 0.2;
    pipeline.fail_closed_on_post_limit_change = true;
    pipeline.max_post_limit_delta_v = 1e-6;
    ASSERT_TRUE(
        fixture.engine.configureCommandPipeline(pipeline, fixture.error))
        << fixture.error;

    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "OK";
    fixture.solver.next_output.cmd_v = 1.0;
    const ControlCycleResult cycle = fixture.engine.step(fixture.request());
    const CommandPipelineResult& publication = cycle.publication.pipeline;

    EXPECT_TRUE(publication.linear_limited);
    EXPECT_TRUE(publication.command_contract_violation);
    EXPECT_EQ(publication.decision.source,
              CommandSource::ExecutionContract);
    EXPECT_DOUBLE_EQ(publication.final_command.linear, 0.0);

    fixture.sink.now_ns = secondsToNanoseconds(1.1);
    const CommandPublicationResult waiting =
        fixture.engine.publishFailClosedZero(
            2, secondsToNanoseconds(1.0), 0.1,
            &fixture.sink, &fixture.history, true,
            "WAITING_FOR_ODOM");
    EXPECT_EQ(waiting.pipeline.decision.source, CommandSource::FailClosed);
    EXPECT_EQ(waiting.pipeline.decision.reason, "WAITING_FOR_ODOM");
    EXPECT_DOUBLE_EQ(waiting.pipeline.final_command.linear, 0.0);
}

TEST(ControlCycleEngineTest, CommitsPhaseOnlyAfterConsistentReceipt) {
    EngineFixture fixture;
    configureMonitorPhase(fixture);
    configureAcceptedMonitorSolve(fixture);
    ControlCycleRequest request = monitorRequest(fixture);

    fixture.sink.deliver = false;
    ControlCycleResult failed = fixture.engine.step(request);
    EXPECT_EQ(1, fixture.sink.calls);
    EXPECT_FALSE(failed.phase_committed);
    EXPECT_FALSE(fixture.engine.phaseRejoinCoordinator().haveAcceptedIndex());
    EXPECT_TRUE(fixture.history.empty());

    fixture.sink.deliver = true;
    fixture.sink.now_ns = secondsToNanoseconds(1.1);
    request.cycle_id = 2;
    request.phase_time_sec = 1.1;
    ControlCycleResult delivered = fixture.engine.step(request);
    EXPECT_EQ(2, fixture.sink.calls);
    EXPECT_TRUE(delivered.publication.published());
    EXPECT_TRUE(delivered.phase_committed);
    EXPECT_TRUE(fixture.engine.phaseRejoinCoordinator().haveAcceptedIndex());
    EXPECT_TRUE(delivered.publication.history_committed);
}

TEST(ControlCycleEngineTest, AuditsExpectedAndActualPublishEpoch) {
    EngineFixture fixture;
    PublishLatencyModelConfig latency;
    latency.enabled = true;
    latency.estimated_dc_sec = 0.05;
    ASSERT_TRUE(fixture.engine.configurePublishLatency(
        latency, fixture.error)) << fixture.error;
    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "OK";
    fixture.solver.next_output.cmd_v = 0.2;

    ControlCycleRequest request = fixture.request();
    fixture.sink.now_ns = secondsToNanoseconds(0.97);
    const ControlCycleResult result = fixture.engine.step(request);

    EXPECT_TRUE(result.publication.publish_timing.estimate.valid);
    EXPECT_EQ(secondsToNanoseconds(0.95),
              result.publication.publish_timing.estimate
                  .expected_publish_stamp_ns);
    EXPECT_TRUE(result.publication.publish_timing.actual_valid);
    EXPECT_NEAR(0.07,
                result.publication.publish_timing.actual_dc_sec,
                1e-12);
    EXPECT_NEAR(0.02,
                result.publication.publish_timing.dc_error_sec,
                1e-12);
    EXPECT_FALSE(
        result.publication.publish_timing.publish_deadline_missed);
    EXPECT_TRUE(result.telemetry.publish_timing.actual_valid);
}

TEST(ControlCycleEngineTest, LimiterRewriteBlocksPhaseCommit) {
    EngineFixture fixture;
    configureMonitorPhase(fixture);
    configureAcceptedMonitorSolve(fixture);
    CommandPipelineConfig pipeline;
    pipeline.control_frequency = 10.0;
    pipeline.linear_accel_limit_enable = true;
    pipeline.linear_accel_max = 1.0;
    pipeline.linear_accel_max_dt = 0.2;
    ASSERT_TRUE(fixture.engine.configureCommandPipeline(
        pipeline, fixture.error)) << fixture.error;

    const ControlCycleResult result = fixture.engine.step(
        monitorRequest(fixture));
    EXPECT_TRUE(result.publication.published());
    EXPECT_TRUE(result.publication.pipeline.linear_limited);
    EXPECT_FALSE(result.phase_committed);
    EXPECT_FALSE(fixture.engine.phaseRejoinCoordinator().haveAcceptedIndex());
}

TEST(ControlCycleEngineTest, TerminalClampHasExplicitPriorityOverRawSolver) {
    EngineFixture fixture;
    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "TERMINAL_SLOWDOWN";
    fixture.solver.next_output.cmd_v = 0.2;
    fixture.solver.next_output.first_shot_debug.success = true;
    fixture.solver.next_output.first_shot_debug.cmd_v_post_clamp = 0.7;

    const ControlCycleResult result = fixture.engine.step(fixture.request());
    EXPECT_TRUE(result.terminal_controller_intervened);
    EXPECT_EQ(CommandSource::Terminal, result.decision.source);
    EXPECT_DOUBLE_EQ(0.2, result.output.cmd_v);
}

TEST(ControlCycleEngineTest, GoalReachedLatchPersistsUntilReferenceReset) {
    EngineFixture fixture;
    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "GOAL_REACHED";
    fixture.solver.next_output.terminal_diagnostics.reached = true;
    ControlCycleResult result = fixture.engine.step(fixture.request());
    EXPECT_EQ(CommandSource::Terminal, result.decision.source);
    EXPECT_EQ("GOAL_REACHED", result.output.status);

    fixture.solver.next_output = SolverOutput{};
    fixture.solver.next_output.success = false;
    fixture.solver.next_output.status = "SOLVER_FAILED";
    result = fixture.engine.step(fixture.request());
    EXPECT_TRUE(result.output.success);
    EXPECT_EQ("GOAL_REACHED_LATCHED", result.output.status);
    EXPECT_DOUBLE_EQ(0.0, result.output.cmd_v);

    fixture.engine.resetForReference();
    result = fixture.engine.step(fixture.request());
    EXPECT_FALSE(result.output.success);
    EXPECT_EQ(CommandSource::FailClosed, result.decision.source);
}

TEST(ControlCycleEngineTest, SafetyOverridesTerminalAndSolverCandidates) {
    FakeSolverSession solver;
    ControlCycleEngine engine(solver);
    PhaseRejoinParams phase;
    phase.mode = PhaseRejoinMode::Off;
    std::string error;
    ASSERT_TRUE(engine.configurePhaseRejoin(phase, error)) << error;
    SafetySupervisorConfig safety;
    safety.terminal_spin.enable = false;
    safety.tracking.spin_enable = false;
    safety.tracking.max_projection_duration_sec = 0.1;
    ASSERT_TRUE(engine.configureSafety(safety, error)) << error;

    solver.next_output.success = true;
    solver.next_output.status = "OK";
    solver.next_output.cmd_v = 0.3;
    solver.next_output.projector_debug.raw_valid = true;
    solver.next_output.projector_debug.raw_distance = 0.8;
    ControlCycleRequest request;
    request.period_sec = 0.1;
    FakeCommandSink sink;
    CommandHistoryBuffer history;
    request.command_sink = &sink;
    request.command_history = &history;
    const ControlCycleResult result = engine.step(request);

    EXPECT_FALSE(result.output.success);
    EXPECT_TRUE(result.safety.blocked);
    EXPECT_EQ(CommandSource::Safety, result.decision.source);
    EXPECT_EQ("TRACKING_UNSAFE_PROJECTION", result.output.status);
    EXPECT_DOUBLE_EQ(0.0, result.output.cmd_v);
    EXPECT_EQ(result.phase_debug.status,
              "SAFETY_OVERRIDE_TRACKING_UNSAFE_PROJECTION");
}

TEST(ControlCycleEngineTest, PhaseOffInjectsExplicitOffSolverContext) {
    EngineFixture fixture;
    fixture.solver.next_output.success = true;
    fixture.solver.next_output.status = "OK";
    const ControlCycleResult result = fixture.engine.step(fixture.request());
    EXPECT_EQ("OFF", result.phase_preparation.status);
    EXPECT_FALSE(fixture.solver.last_input.phase_rejoin.active);
}

TEST(ControlCycleEngineTest, ReplaysFrozenCommandAndStatusGoldenCycles) {
    FakeSolverSession solver;
    ControlCycleEngine engine(solver);
    PhaseRejoinParams phase;
    phase.mode = PhaseRejoinMode::Off;
    std::string error;
    ASSERT_TRUE(engine.configurePhaseRejoin(phase, error)) << error;
    SafetySupervisorConfig safety;
    safety.terminal_spin.enable = false;
    safety.tracking.spin_enable = false;
    safety.tracking.max_projection_duration_sec = 0.2;
    ASSERT_TRUE(engine.configureSafety(safety, error)) << error;
    CommandPipelineConfig pipeline;
    pipeline.linear_accel_limit_enable = false;
    pipeline.angular_limit_enable = false;
    ASSERT_TRUE(engine.configureCommandPipeline(pipeline, error)) << error;
    FakeCommandSink sink;
    CommandHistoryBuffer history;

    std::ifstream input(goldenPath());
    ASSERT_TRUE(input.is_open());
    std::string line;
    ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
    std::size_t replayed = 0;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        const std::vector<std::string> field = splitCsv(line);
        ASSERT_EQ(19u, field.size()) << line;
        const std::uint64_t cycle_id = std::stoull(field[0]);
        if (std::stoi(field[1]) != 0) {
            engine.resetForReference();
        }

        solver.next_output = SolverOutput{};
        solver.next_output.success = std::stoi(field[2]) != 0;
        solver.next_output.status = field[3];
        solver.next_output.first_shot_debug.success =
            solver.next_output.success;
        solver.next_output.first_shot_debug.cmd_v_post_clamp =
            std::stod(field[4]);
        solver.next_output.first_shot_debug.cmd_omega_post_clamp =
            std::stod(field[5]);
        solver.next_output.cmd_v = std::stod(field[6]);
        solver.next_output.cmd_omega = std::stod(field[7]);
        solver.next_output.terminal_diagnostics.terminal_phase =
            std::stoi(field[8]) != 0;
        solver.next_output.terminal_diagnostics.reached =
            std::stoi(field[9]) != 0;
        solver.next_output.projector_debug.raw_valid =
            std::stoi(field[10]) != 0;
        solver.next_output.projector_debug.raw_distance =
            std::stod(field[11]);

        ControlCycleRequest request;
        request.cycle_id = cycle_id;
        request.solver_input.robot.omega = std::stod(field[12]);
        request.period_sec = std::stod(field[13]);
        sink.now_ns = secondsToNanoseconds(
            1.0 + 0.1 * static_cast<double>(cycle_id));
        request.command_sink = &sink;
        request.command_history = &history;
        const ControlCycleResult result = engine.step(request);

        EXPECT_EQ(field[14], commandSourceName(result.decision.source))
            << "cycle=" << cycle_id;
        EXPECT_EQ(std::stoi(field[15]) != 0, result.output.success)
            << "cycle=" << cycle_id;
        EXPECT_NEAR(std::stod(field[16]), result.output.cmd_v, 1e-12)
            << "cycle=" << cycle_id;
        EXPECT_NEAR(std::stod(field[17]), result.output.cmd_omega, 1e-12)
            << "cycle=" << cycle_id;
        EXPECT_EQ(field[18], result.output.status)
            << "cycle=" << cycle_id;
        ++replayed;
    }
    EXPECT_EQ(8u, replayed);
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
