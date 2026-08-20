#include <gtest/gtest.h>

#include "spmpc_local_planner/controller/control_cycle_engine.h"

#include <fstream>
#include <sstream>
#include <vector>

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

struct EngineFixture {
    EngineFixture() : engine(solver) {
        PhaseRejoinParams phase;
        phase.mode = PhaseRejoinMode::Off;
        EXPECT_TRUE(engine.configurePhaseRejoin(phase, error)) << error;
        EXPECT_TRUE(engine.configureSafety(safety, error)) << error;
    }

    ControlCycleRequest request() const {
        ControlCycleRequest request;
        request.cycle_id = 1;
        request.solver_input.dt = 0.1;
        request.solver_input.horizon_steps = 10;
        request.period_sec = 0.1;
        return request;
    }

    FakeSolverSession solver;
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
    const ControlCycleResult result = engine.step(request);

    EXPECT_FALSE(result.output.success);
    EXPECT_TRUE(result.safety.blocked);
    EXPECT_EQ(CommandSource::Safety, result.decision.source);
    EXPECT_EQ("TRACKING_UNSAFE_PROJECTION", result.output.status);
    EXPECT_DOUBLE_EQ(0.0, result.output.cmd_v);
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
