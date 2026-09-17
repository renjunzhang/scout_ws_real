#include "spmpc_local_planner/core/spmpc_problem.h"
#include <gtest/gtest.h>
#include <limits>

using namespace spmpc_local_planner;
namespace {
class FailingSolver : public SpmpcSolver {
public:
    explicit FailingSolver(bool recoverable) : recoverable_(recoverable) {}
    void configure(const SolverParams&, const VariantConfig&) override {}
    bool solve(const SolverInput&, const ReferencePath&, SolverOutput& output) const override {
        output = SolverOutput{};
        output.recoverable_solver_failure = recoverable_;
        output.status = recoverable_ ? "ACADOS_SOLVE_FAILED_4" : "INVALID_ACTUATOR_STATE";
        return false;
    }
private:
    bool recoverable_;
};

class HoldCommandSolver : public SpmpcSolver {
public:
    void configure(const SolverParams&, const VariantConfig&) override {}
    bool solve(const SolverInput& input, const ReferencePath&, SolverOutput& output) const override {
        output = SolverOutput{};
        output.success = true;
        output.predicted_horizon.valid = true;
        output.cmd_v = input.actuator.v_cmd;
        output.cmd_omega = input.actuator.omega_cmd;
        return true;
    }
};

class BudgetThenHoldSolver : public HoldCommandSolver {
public:
    std::string failure_status = "SOLVE_BUDGET_EXHAUSTED";
    mutable int attempts = 0;
    mutable bool remeasured = false;
    bool solve(const SolverInput& input, const ReferencePath& path, SolverOutput& output) const override {
        remeasured = input.solve_budget.remeasure_first_iteration;
        if (++attempts == 1) {
            output = SolverOutput{};
            output.recoverable_solver_failure = true;
            output.status = failure_status;
            return false;
        }
        return HoldCommandSolver::solve(input, path, output);
    }
};

SolverParams params() {
    SolverParams p;
    p.jerk_limit_enable = true;
    p.terminal.mpc_stop_handoff_enable = true;
    p.planning.region.enabled = true;
    p.planning.region.id = "stop-test";
    p.planning.region.cells = {{"room",0.,10.,{{-2.,-2.},{12.,-2.},{12.,2.},{-2.,2.}}}};
    return p;
}
void configure(SpmpcProblem& problem) {
    VariantConfig b0;
    b0.slosh_enable = false;
    b0.w_slosh = 0.;
    problem.configure(params(), b0);
    ReferencePath reference;
    reference.setPoints({{0,0,0,0,0},{10,0,0,0,0}},"map");
    problem.setReferencePath(reference);
}
SolverInput moving() {
    SolverInput input;
    input.robot.x = 1.;
    input.robot.v = .2036;
    input.actuator.valid = true;
    input.actuator.v_cmd = .2;
    input.actuator.delayed_v_cmd = .2;
    input.actuator.linear_delay_queue.fill(.2);
    input.actuator.angular_delay_queue.fill(0.);
    return input;
}
}

TEST(SolverFailureStop, NumericalFailureUsesCurrentVerifiedTailAndKeepsOwnership) {
    SpmpcProblem problem(std::make_unique<FailingSolver>(true));
    configure(problem);
    ASSERT_TRUE(problem.configurationError().empty()) << problem.configurationError();
    auto input = moving();
    SolverOutput output;
    ASSERT_TRUE(problem.solve(input,output)) << output.status;
    EXPECT_TRUE(output.ocp_solve_attempted);
    EXPECT_TRUE(output.recoverable_solver_failure);
    EXPECT_EQ(output.status,"SOLVER_FAILURE_STOPPING: ACADOS_SOLVE_FAILED_4");
    EXPECT_TRUE(output.terminal_diagnostics.predicted_tail_valid);
    EXPECT_NEAR(output.cmd_v,.2-1./900.,1e-10);
    EXPECT_FALSE(output.predicted_horizon.valid);
    ASSERT_TRUE(problem.solve(input,output)) << output.status;
    EXPECT_FALSE(output.ocp_solve_attempted);
    EXPECT_GT(output.cmd_v,0.);
}

TEST(SolverFailureStop, InvalidStateAndUnsafeTailCannotUseFallback) {
    SpmpcProblem invalid(std::make_unique<FailingSolver>(false));
    configure(invalid);
    SolverOutput output;
    EXPECT_FALSE(invalid.solve(moving(),output));
    EXPECT_EQ(output.status,"INVALID_ACTUATOR_STATE");
    SpmpcProblem unsafe(std::make_unique<FailingSolver>(true));
    configure(unsafe);
    auto input=moving();
    input.actuator.linear_delay_queue.front()=1.2;
    EXPECT_FALSE(unsafe.solve(input,output));
    EXPECT_FALSE(output.success);
    EXPECT_FALSE(output.terminal_diagnostics.predicted_tail_valid);
    input=moving(); input.actuator.valid=false;
    EXPECT_FALSE(unsafe.solve(input,output));
    EXPECT_FALSE(output.success);
}

TEST(SolverFailureStop, BudgetStopRetriesOnlyAfterVehicleAndCommandsClear) {
    auto solver = std::make_unique<BudgetThenHoldSolver>();
    auto* observed = solver.get();
    SpmpcProblem problem(std::move(solver));
    configure(problem);
    auto input = moving();
    SolverOutput output;
    ASSERT_TRUE(problem.solve(input, output)) << output.status;
    EXPECT_EQ(output.status, "SOLVER_FAILURE_STOPPING: SOLVE_BUDGET_EXHAUSTED");
    EXPECT_GT(output.cmd_v, 0.);  // certified braking, not an abrupt zero
    ASSERT_TRUE(problem.solve(input, output)) << output.status;
    EXPECT_FALSE(output.ocp_solve_attempted);
    EXPECT_EQ(output.status, "SOLVE_BUDGET_STOPPING");
    input.robot.v = 0.;
    ASSERT_TRUE(problem.solve(input, output)) << output.status;
    EXPECT_EQ(observed->attempts, 1);  // delayed commands still in flight
    input.actuator = ActuatorState{};
    input.actuator.valid = true;
    input.robot.v = .04;
    ASSERT_TRUE(problem.solve(input, output)) << output.status;
    EXPECT_EQ(observed->attempts, 1);  // measured motion has not stopped
    input.robot.v = 0.;
    ASSERT_TRUE(problem.solve(input, output)) << output.status;
    EXPECT_TRUE(output.ocp_solve_attempted);
    EXPECT_EQ(observed->attempts, 2);
    EXPECT_TRUE(observed->remeasured);
    EXPECT_FALSE(output.terminal_diagnostics.reached);  // still far from goal
    ASSERT_TRUE(problem.solve(input, output)) << output.status;
    EXPECT_EQ(observed->attempts, 3);
    EXPECT_FALSE(observed->remeasured);
}

TEST(SolverFailureStop, RejectedPredictionBrakesWithVerifiedTailThenRetries) {
    auto solver=std::make_unique<BudgetThenHoldSolver>();
    auto* observed=solver.get();
    observed->failure_status="PREDICTION_DYNAMICS_VIOLATION";
    SpmpcProblem problem(std::move(solver));configure(problem);
    auto input=moving();SolverOutput output;
    ASSERT_TRUE(problem.solve(input,output));
    EXPECT_GT(output.cmd_v,0.);
    EXPECT_TRUE(output.terminal_diagnostics.predicted_tail_valid);
    EXPECT_FALSE(output.terminal_diagnostics.command_owned);
    ASSERT_TRUE(problem.solve(input,output));
    EXPECT_EQ(output.status,"SOLVER_RETRY_STOPPING: PREDICTION_DYNAMICS_VIOLATION");
    EXPECT_EQ(observed->attempts,1);
    input.robot.v=0.;input.actuator=ActuatorState{};input.actuator.valid=true;
    ASSERT_TRUE(problem.solve(input,output));
    EXPECT_EQ(observed->attempts,2);
    EXPECT_TRUE(output.ocp_solve_attempted);
}

TEST(SolverFailureStop, BudgetRecoveryDoesNotReleaseGoalHandoffOrUnsafeTail) {
    auto solver = std::make_unique<BudgetThenHoldSolver>();
    auto* observed = solver.get();
    SpmpcProblem problem(std::move(solver));
    configure(problem);
    ReferencePath short_path;
    short_path.setPoints({{0,0,0,0,0},{1.5,0,0,0,0}}, "map");
    problem.setReferencePath(short_path);
    SolverOutput output;
    auto input = moving();
    ASSERT_TRUE(problem.solve(input, output));
    input.actuator.linear_delay_queue.front() = 1.2;
    EXPECT_FALSE(problem.solve(input, output));
    EXPECT_EQ(observed->attempts, 1);
    input = moving();
    input.robot.x = 1.5;
    input.robot.v = 0.;
    input.actuator = ActuatorState{};
    input.actuator.valid = true;
    ASSERT_TRUE(problem.solve(input, output)) << output.status;
    EXPECT_EQ(output.status, "GOAL_REACHED");
    EXPECT_FALSE(output.ocp_solve_attempted);
    input.robot.x = 1.;  // localization correction cannot undo terminal ownership
    input.robot.v = 0.;
    input.actuator = ActuatorState{};
    input.actuator.valid = true;
    ASSERT_TRUE(problem.solve(input, output)) << output.status;
    EXPECT_EQ(observed->attempts, 1);
    EXPECT_FALSE(output.ocp_solve_attempted);
}

TEST(SolverFailureStop, LiquidCapUsesIntegrationSubstepsAndRespectsEnableSwitch) {
    auto p=params();
    p.slosh.slosh_height_max=.0009;
    p.liquid_limit.recovery_enable=true;
    p.liquid_limit.recovery_budget_m=.0001;
    p.liquid_limit.freeboard_m=.001;
    VariantConfig full;
    full.slosh_enable=true;
    full.slosh_constraint_enable=true;
    auto input=moving(); input.robot.x=10.; input.robot.v=.0509;
    input.actuator.v_cmd=input.actuator.delayed_v_cmd=.05;
    input.actuator.linear_delay_queue.fill(.05);
    input.slosh.eta_x=.000547272529555;
    input.slosh.eta_x_dot=.00474505488508;
    ReferencePath reference;
    reference.setPoints({{0,0,0,0,0},{10,0,0,0,0}},"map");
    SpmpcProblem problem(std::make_unique<FailingSolver>(true));
    problem.configure(p,full); problem.setReferencePath(reference);
    SolverOutput output;
    EXPECT_FALSE(problem.solve(input,output));
    EXPECT_EQ(output.status,"STOP_LIQUID_RECOVERY_CAP");
    EXPECT_GT(output.terminal_diagnostics.predicted_tail_peak_height_m,.001);
    EXPECT_FALSE(output.ocp_solve_attempted);
    // A liquid cost alone never turns its performance target into a hard cap.
    full.slosh_constraint_enable=false;
    problem.configure(p,full); problem.setReferencePath(reference);
    EXPECT_TRUE(problem.solve(input,output)) << output.status;
}

TEST(SolverFailureStop, RejectedCandidateCannotFallBackToLiquidUnsafeTailWithoutRegion) {
    auto p = params();
    p.planning.region.enabled = false;
    p.task_stop.enable = true;
    p.slosh.slosh_height_max = .001;
    VariantConfig full;
    full.slosh_enable = true;
    full.slosh_constraint_enable = true;
    SpmpcProblem problem(std::make_unique<HoldCommandSolver>());
    problem.configure(p, full);
    ASSERT_TRUE(problem.configurationError().empty()) << problem.configurationError();
    ReferencePath reference;
    reference.setPoints({{0,0,0,0,0},{10,0,0,0,0}}, "map");
    problem.setReferencePath(reference);
    auto input = moving();
    input.slosh.eta_x = .01;
    SolverOutput output;
    EXPECT_FALSE(problem.solve(input, output));
    EXPECT_TRUE(output.ocp_solve_attempted);
    EXPECT_FALSE(output.success);
    EXPECT_EQ(output.status, "STOP_LIQUID_RECOVERY_CAP");
    EXPECT_DOUBLE_EQ(output.cmd_v, 0.0);
    EXPECT_FALSE(output.predicted_horizon.valid);
}
