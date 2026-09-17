#include "spmpc_local_planner/core/task_stop_manager.h"
#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/dynamics/actual_motion_propagator.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace spmpc_local_planner {

namespace {
const StopMotionLimits kStopLimits{-0.002, 0.8, 1.2};

class BrakingSolver : public SpmpcSolver {
public:
    void configure(const SolverParams& params, const VariantConfig&) override { params_=params; }
    bool solve(const SolverInput& input, const ReferencePath&, SolverOutput& output) const override {
        EXPECT_TRUE(input.task_stop_active);
        EXPECT_TRUE(std::isfinite(input.terminal_v_cap));
        const auto stop=makeJerkLimitedStopCommand(input.actuator,input.dt,
            params_.a_max,params_.alpha_max,params_.jerk_max);
        output=SolverOutput{};
        output.success=stop.valid;output.status="TEST_BRAKING_OCP";
        output.cmd_v=stop.v;output.cmd_omega=stop.omega;
        return stop.valid;
    }
private:
    SolverParams params_;
};

MotionRegion stopRegion(double right_edge, double footprint = .426,
                        double margin = .02) {
    MotionRegionConfig config;
    config.enabled = true;
    config.id = "stop-test";
    config.frame_id = "map";
    config.footprint_radius = footprint;
    config.margin = margin;
    config.cells.push_back({"only", 0., 10.,
        {{-1., -1.}, {right_edge, -1.}, {right_edge, 1.}, {-1., 1.}}});
    return MotionRegion(config);
}

SolverInput stopInput(double x, double v, double omega = 0.) {
    SolverInput input;
    input.robot.x = x;
    input.robot.v = v;
    input.robot.omega = omega;
    input.actuator.valid = true;
    input.actuator.v_cmd = v;
    input.actuator.omega_cmd = omega;
    input.actuator.linear_delay_queue.fill(v);
    input.actuator.angular_delay_queue.fill(omega);
    return input;
}
}

TEST(TaskStop, OrdinaryMpccDrainsCommandsWithJerkWithoutLiquidState) {
    SolverParams params;
    params.terminal.enable=true;
    params.terminal.mpc_stop_handoff_enable=true;
    params.jerk_limit_enable=true;
    params.task_stop.enable=false;
    VariantConfig variant; variant.slosh_enable=false; variant.w_slosh=0;
    SpmpcProblem problem(std::make_unique<BrakingSolver>()); problem.configure(params, variant);
    EXPECT_FALSE(problem.requiresLiquidState());
    ReferencePath route;
    route.setPoints({{0,0,0,0,0},{1,0,0,0,0}},"map");
    problem.setReferencePath(route);
    SolverInput input; input.robot.x=1;
    input.actuator.valid=true; input.actuator.v_cmd=.1;
    input.actuator.linear_delay_queue.fill(.1);
    input.slosh.eta_x=std::numeric_limits<double>::quiet_NaN();
    SolverOutput output;
    ASSERT_TRUE(problem.solve(input,output)) << output.status;
    EXPECT_EQ(output.status,"TERMINAL_DRAINING");
    EXPECT_FALSE(output.ocp_solve_attempted);
    EXPECT_TRUE(output.terminal_diagnostics.command_owned);
    EXPECT_TRUE(output.terminal_diagnostics.predicted_tail_valid);
    EXPECT_FALSE(output.terminal_diagnostics.reached);
    const double a=(output.cmd_v-input.actuator.v_cmd)/input.dt;
    EXPECT_LE(std::abs(a-input.actuator.a_cmd_memory),params.jerk_max*input.dt+1e-9);
    input.actuator.v_cmd=0;
    ASSERT_TRUE(problem.solve(input,output));
    EXPECT_NE(output.status,"GOAL_REACHED"); // Last queued command still excites the vehicle.
    input.actuator.linear_delay_queue.fill(0);
    ASSERT_TRUE(problem.solve(input,output));
    EXPECT_EQ(output.status,"GOAL_REACHED");
}

TEST(TaskStop, QueuedMotionThatWouldLeaveGoalCannotLatchNormalHandoff) {
    SolverParams params;
    params.terminal.mpc_stop_handoff_enable=true;
    params.terminal.goal_tolerance=.05;
    params.jerk_limit_enable=true;
    VariantConfig variant;variant.slosh_enable=false;variant.w_slosh=0;
    SpmpcProblem problem(std::make_unique<BrakingSolver>());
    problem.configure(params,variant);
    ReferencePath route;route.setPoints({{0,0,0,0,0},{1,0,0,0,0}},"map");
    problem.setReferencePath(route);
    auto input=stopInput(.99,0.);
    input.actuator.v_cmd=input.actuator.delayed_v_cmd=.3;
    input.actuator.linear_delay_queue.fill(.3);
    SolverOutput output;
    ASSERT_TRUE(problem.solve(input,output))<<output.status;
    EXPECT_TRUE(output.ocp_solve_attempted);
    EXPECT_FALSE(output.terminal_diagnostics.command_owned);
    EXPECT_FALSE(output.terminal_diagnostics.reached);
}

TEST(TaskStop, OrdinaryReachedRechecksPoseMotionAndQueuesWithoutRestarting) {
    SolverParams params;
    params.terminal.mpc_stop_handoff_enable=true;
    params.terminal.goal_tolerance=.05;
    params.terminal.require_goal_yaw=true;
    params.jerk_limit_enable=true;
    VariantConfig variant; variant.slosh_enable=false; variant.w_slosh=0;
    SpmpcProblem problem; problem.configure(params,variant);
    ReferencePath route; route.setPoints({{0,0,0,0,0},{.55,0,0,0,0}},"map");
    problem.setReferencePath(route);
    SolverInput input; input.robot.x=.55; input.actuator.valid=true;
    SolverOutput output;
    ASSERT_TRUE(problem.solve(input,output));
    ASSERT_EQ(output.status,"GOAL_REACHED");
    const auto expect_stopping=[&]() {
        ASSERT_TRUE(problem.solve(input,output)) << output.status;
        EXPECT_FALSE(output.terminal_diagnostics.reached);
        EXPECT_TRUE(output.terminal_diagnostics.command_owned);
        EXPECT_NE(output.status,"GOAL_REACHED");
        EXPECT_FALSE(output.ocp_solve_attempted);
        EXPECT_DOUBLE_EQ(output.cmd_v,0.);
        EXPECT_DOUBLE_EQ(output.cmd_omega,0.);
    };
    input.robot.x=.35; expect_stopping(); // Original 200 mm displacement counterexample.
    input.robot.x=.55; input.robot.yaw=.2; expect_stopping();
    input.robot.yaw=0; input.robot.v=.1; expect_stopping();
    input.robot.v=0; input.robot.omega=.2; expect_stopping();
    input.robot.omega=0; input.actuator.linear_delay_queue.front()=.1; expect_stopping();
    input.actuator.linear_delay_queue.fill(0);
    ASSERT_TRUE(problem.solve(input,output));
    EXPECT_EQ(output.status,"GOAL_REACHED");
}

TEST(TaskStop, BrakeAndReleaseRespectCommandJerkThroughZeroSpeed) {
    ActuatorState state;state.valid=true;state.v_cmd=.2;state.omega_cmd=-.3;state.a_cmd_memory=.15;
    const double dt=1./30,jerk=1;
    bool stopped=false;
    for(int k=0;k<180;++k) {
        const auto out=makeJerkLimitedStopCommand(state,dt,.6,1.2,jerk);
        ASSERT_TRUE(out.valid)<<out.status;
        EXPECT_GE(out.v,0.);
        EXPECT_LE(std::abs(out.a),.6+1e-10);
        EXPECT_LE(std::abs(out.a-state.a_cmd_memory),jerk*dt+1e-10);
        EXPECT_NEAR((out.v-state.v_cmd)/dt,out.a,1e-12);
        EXPECT_LE(std::abs(out.omega-state.omega_cmd),1.2*dt+1e-10);
        state.v_cmd=out.v;state.omega_cmd=out.omega;state.a_cmd_memory=out.a;
        if(out.v<1e-12 && std::abs(out.a)<1e-12 && out.omega==0){stopped=true;break;}
    }
    EXPECT_TRUE(stopped);
    EXPECT_DOUBLE_EQ(makeJerkLimitedStopCommand(state,dt,.6,1.2,jerk).v,0.);
}

TEST(TaskStop, InconsistentHistoryFailsWithoutInventingMotion) {
    ActuatorState state;state.valid=true;state.v_cmd=0;state.a_cmd_memory=-.07;
    const auto out=makeJerkLimitedStopCommand(state,1./30,.6,1.2,1.);
    EXPECT_FALSE(out.valid);
    EXPECT_EQ(out.status,"STOP_JERK_HISTORY_INFEASIBLE");
    EXPECT_DOUBLE_EQ(out.v,0.);
    state.a_cmd_memory=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(makeJerkLimitedStopCommand(state,1./30,.6,1.2,1.).valid);
}

TEST(TaskStop, TailIncludesDelayedCommandsActualMotionAndLiquid) {
    TaskStopManager manager;
    ASSERT_TRUE(manager.configure({}, {}, {}, kStopLimits, .6,1.2,1.));
    SolverInput input;input.robot.v=.2;input.robot.omega=.3;input.actuator.valid=true;
    input.actuator.v_cmd=.2;input.actuator.omega_cmd=.3;
    input.actuator.linear_delay_queue.fill(.2);input.actuator.angular_delay_queue.fill(.3);
    const auto tail=manager.predict(input);
    ASSERT_TRUE(tail.valid)<<tail.status;
    EXPECT_GT(tail.distance_m,.2*.2/(2*.6));
    EXPECT_GT(tail.duration_sec,kExplicitAngularDelaySteps*input.dt);
    EXPECT_GT(tail.peak_height_m,1e-6);
    EXPECT_GT(tail.residual_height_m,0.);
    EXPECT_DOUBLE_EQ(input.actuator.linear_delay_queue.front(),.2);
    EXPECT_DOUBLE_EQ(input.robot.v,.2);
}

TEST(TaskStop, TailReportsAllMotionAndCommandBoundsForNormalStop) {
    TaskStopManager manager;
    ASSERT_TRUE(manager.configure({}, {}, {}, kStopLimits, .6, 1.2, 1.));
    const auto tail = manager.predict(stopInput(0., .2, .3));
    ASSERT_TRUE(tail.valid) << tail.status;
}

TEST(TaskStop, TailRejectsCommandAndActualMotionBoundaryViolations) {
    TaskStopManager manager;
    ASSERT_TRUE(manager.configure({}, {}, {}, kStopLimits, .6, 1.2, 1.));

    auto command_input = stopInput(0., .2);
    command_input.actuator.linear_delay_queue.front() = .81;
    auto tail = manager.predict(command_input);
    EXPECT_FALSE(tail.valid);
    EXPECT_EQ(tail.status, "STOP_COMMAND_FIFO_BOUND_VIOLATION");

    auto actual_input = stopInput(0., .2);
    actual_input.robot.v = .81;
    tail = manager.predict(actual_input);
    EXPECT_FALSE(tail.valid);
    EXPECT_EQ(tail.status, "STOP_ACTUAL_MOTION_BOUND_VIOLATION");
}

TEST(TaskStop, InitiallyLegalHistoryCannotHideLaterVelocityOvershoot) {
    TaskStopManager manager;
    ASSERT_TRUE(manager.configure({}, {}, {}, kStopLimits, .6, 1.2, 1.));
    auto input=stopInput(0., .8);
    input.robot.v=.79;
    input.actuator.delayed_v_cmd=.8;
    auto tail=manager.predict(input);
    EXPECT_FALSE(tail.valid);
    EXPECT_EQ(tail.status,"STOP_ACTUAL_MOTION_BOUND_VIOLATION");
    input.actuator.v_cmd=.76;
    input.actuator.a_cmd_memory=.4;
    input.actuator.linear_delay_queue={{.7066666666667,.72,.7333333333333,.7466666666667,.76}};
    input.actuator.delayed_v_cmd=input.actuator.linear_delay_queue.front();
    tail=manager.predict(input);
    EXPECT_FALSE(tail.valid);
    EXPECT_NE(tail.status.find("BOUND_VIOLATION"),std::string::npos);
}

TEST(TaskStop, ActualMotionDiagnosticsIncludeEveryGeneratedSubstep) {
    SloshModelParams liquid;
    SloshDynamics model;
    ASSERT_TRUE(model.configure(liquid));
    auto input = stopInput(0., .2);
    input.slosh.eta_x = .00099 / model.heightCoeff();
    input.slosh.eta_x_dot = .0002 * model.omegaN() / model.heightCoeff();
    RobotState robot = input.robot;
    SloshState state = input.slosh;
    ActualMotionDiagnostics diagnostics;
    ASSERT_TRUE(propagateActualMotion(
        robot, state, {input.actuator.linear_delay_queue.front(), 0.},
        {}, model, input.dt, &diagnostics));
    EXPECT_TRUE(diagnostics.valid);
    EXPECT_EQ(diagnostics.substeps, 4);
    EXPECT_GE(diagnostics.peak_height_m, model.height(state));
}

TEST(TaskStop, QueueDrainAndModalVelocityGateCompletionSeparately) {
    TaskStopManager manager;TaskStopParams params;
    params.stable_hold_sec=.1;
    ASSERT_TRUE(manager.configure(params, {}, {}, kStopLimits, .6,1.2,1.));
    SolverInput input;input.actuator.valid=true;
    input.actuator.angular_delay_queue.back()=.1;
    auto state=manager.observe(input,true,.03,.05);
    EXPECT_FALSE(state.queues_clear);EXPECT_FALSE(state.liquid_stable);
    input.actuator.angular_delay_queue.fill(0);
    input.slosh.eta_x_dot=.1; // Height is zero but the next liquid peak is large.
    state=manager.observe(input,true,.03,.05);
    EXPECT_TRUE(state.vehicle_stopped);EXPECT_TRUE(state.excitation_quiet);
    EXPECT_GT(state.residual_height_m,params.residual_height_m);
    EXPECT_FALSE(state.liquid_stable);
    input.slosh={};
    for(int k=0;k<4;++k) state=manager.observe(input,true,.03,.05);
    EXPECT_TRUE(state.liquid_stable);
    EXPECT_LT(state.vehicle_stop_time_sec,state.liquid_stable_time_sec);
    manager.reset();
    state=manager.observe(input,false,.03,.05);
    EXPECT_FALSE(state.vehicle_stopped);EXPECT_FALSE(state.liquid_stable);
}

TEST(TaskStop, SettleTimeoutDoesNotClaimSuccess) {
    TaskStopManager manager;TaskStopParams params;params.max_settle_sec=.1;
    ASSERT_TRUE(manager.configure(params, {}, {}, kStopLimits, .6,1.2,1.));
    SolverInput input;input.actuator.valid=true;input.slosh.eta_x_dot=.1;
    StopReadiness state;
    for(int k=0;k<6;++k)state=manager.observe(input,true,.03,.05);
    EXPECT_TRUE(state.timed_out);EXPECT_FALSE(state.liquid_stable);
    input.slosh={};
    for(int k=0;k<12;++k)state=manager.observe(input,true,.03,.05);
    EXPECT_TRUE(state.liquid_stable);
    EXPECT_TRUE(state.timed_out);
    EXPECT_LT(state.liquid_stable_time_sec,0.);
    manager.reset();
    state=manager.observe(input,true,.03,.05);
    EXPECT_FALSE(state.timed_out);
}

TEST(TaskStop, CommonEpochGapsDoNotCountAsContinuousLiquidStability) {
    TaskStopManager manager;TaskStopParams params;params.stable_hold_sec=.1;
    ASSERT_TRUE(manager.configure(params, {}, {}, kStopLimits, .6,1.2,1.));
    SolverInput input;input.actuator.valid=true;
    input.cycle_timing.solver_input_epoch_ns=1000000000;
    EXPECT_FALSE(manager.observe(input,true,.03,.05).liquid_stable);
    input.cycle_timing.solver_input_epoch_ns+=33333333;
    EXPECT_FALSE(manager.observe(input,true,.03,.05).liquid_stable);
    EXPECT_FALSE(manager.observe(input,true,.03,.05).valid); // Duplicate epoch.
    input.cycle_timing.solver_input_epoch_ns=2000000000;
    const auto gap=manager.observe(input,true,.03,.05);
    EXPECT_FALSE(gap.liquid_stable);EXPECT_DOUBLE_EQ(gap.stable_duration_sec,0.);
    StopReadiness state;
    for(int k=0;k<4;++k) {input.cycle_timing.solver_input_epoch_ns+=33333333;state=manager.observe(input,true,.03,.05);}
    EXPECT_TRUE(state.liquid_stable);
}

TEST(TaskStop, ReusedMeasurementsCannotAccumulateStableTimeByAdvancingPredictionEpoch) {
    TaskStopManager manager;TaskStopParams p;p.stable_hold_sec=.1;
    ASSERT_TRUE(manager.configure(p, {}, {}, kStopLimits,.6,1.2,1.));
    SolverInput input;input.actuator.valid=true;
    input.cycle_timing.raw_robot_state_stamp_ns=1000000000;
    input.cycle_timing.raw_liquid_state_stamp_ns=1000000000;
    StopReadiness out;
    for (int k=0;k<8;++k) {
        input.cycle_timing.solver_input_epoch_ns=1000000000LL+k*33333333LL;
        out=manager.observe(input,true,.03,.05);
        EXPECT_TRUE(out.valid);
        EXPECT_FALSE(out.liquid_stable);
    }
}

TEST(TaskStop, InvalidObservationBreaksContinuousStableHold) {
    TaskStopManager manager;TaskStopParams p;p.stable_hold_sec=.1;
    ASSERT_TRUE(manager.configure(p, {}, {}, kStopLimits,.6,1.2,1.));
    SolverInput input;input.actuator.valid=true;
    for (int k=0;k<2;++k) EXPECT_FALSE(manager.observe(input,true,.03,.05).liquid_stable);
    input.slosh.eta_x=std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(manager.observe(input,true,.03,.05).valid);
    input.slosh={};
    EXPECT_FALSE(manager.observe(input,true,.03,.05).liquid_stable);
    EXPECT_FALSE(manager.observe(input,true,.03,.05).liquid_stable);
}

TEST(TaskStop, SlowerMeasurementsCanSettleButNoiseCrossingsRestartHold) {
    TaskStopManager manager;TaskStopParams p;p.stable_hold_sec=.15;
    ASSERT_TRUE(manager.configure(p, {}, {}, kStopLimits,.6,1.2,1.));
    SolverInput input;input.actuator.valid=true;
    StopReadiness out;
    for (int k=0;k<10;++k) {
        input.cycle_timing.solver_input_epoch_ns=1000000000LL+k*33333333LL;
        const auto sensor_epoch=1000000000LL+(k*33333333LL/50000000LL)*50000000LL;
        input.cycle_timing.raw_robot_state_stamp_ns=sensor_epoch;
        input.cycle_timing.raw_liquid_state_stamp_ns=sensor_epoch;
        out=manager.observe(input,true,.03,.05);
    }
    EXPECT_TRUE(out.liquid_stable);
    input.robot.v=1.01*p.quiet_v;
    input.cycle_timing.solver_input_epoch_ns+=33333333LL;
    out=manager.observe(input,true,.03,.05);
    EXPECT_FALSE(out.liquid_stable);EXPECT_DOUBLE_EQ(out.stable_duration_sec,0.);
}

TEST(TaskStop, RegionReportsUnavoidablePrefixAtTheBoundary) {
    TaskStopManager manager;
    ASSERT_TRUE(manager.configure({}, {}, {}, kStopLimits, .6, 1.2, 1.));
    const auto region = stopRegion(1.0);
    auto input = stopInput(.53, .2);
    const auto tail = manager.predict(input, &region, 0.);
    EXPECT_FALSE(tail.valid);
    EXPECT_TRUE(tail.region_checked);
    EXPECT_TRUE(tail.fifo_prefix_violation);
    EXPECT_EQ(tail.status, "STOP_FIFO_REGION_VIOLATION");
    EXPECT_LT(tail.minimum_region_clearance_m, 0.);
}

TEST(TaskStop, ProblemRejectsTheTerminalRegionBypassForBothStopModes) {
    for (bool complete : {false,true}) {
        SolverParams params;
        params.solver_backend="continuous_mpcc_acados";
        params.terminal.mpc_stop_handoff_enable=true;
        params.terminal.goal_tolerance=.05;
        params.jerk_limit_enable=true;
        params.task_stop.enable=complete;
        params.planning.task_deadline_sec=12.;
        params.planning.region=stopRegion(1.).config();
        VariantConfig variant; variant.slosh_enable=false; variant.w_slosh=0;
        SpmpcProblem problem(std::make_unique<BrakingSolver>()); problem.configure(params,variant);
        ASSERT_TRUE(problem.configurationError().empty()) << problem.configurationError();
        ReferencePath route; route.setPoints({{0,0,0,0,0},{.55,0,0,0,0}},"map");
        problem.setReferencePath(route);
        auto input=stopInput(.53,.2);
        input.has_task_elapsed=true; input.task_elapsed_sec=1.;
        SolverOutput output;
        EXPECT_FALSE(problem.solve(input,output));
        EXPECT_FALSE(output.success);
        EXPECT_FALSE(output.ocp_solve_attempted);
        EXPECT_FALSE(output.terminal_diagnostics.reached);
        EXPECT_TRUE(output.terminal_diagnostics.stop_region_checked);
        EXPECT_TRUE(output.terminal_diagnostics.stop_fifo_prefix_violation);
        EXPECT_EQ(output.status,"STOP_FIFO_REGION_VIOLATION");

        params.planning.region=stopRegion(3.).config();
        problem.configure(params,variant); problem.setReferencePath(route);
        ASSERT_TRUE(problem.solve(input,output)) << output.status;
        EXPECT_EQ(output.status,"TEST_BRAKING_OCP");
        EXPECT_TRUE(output.ocp_solve_attempted);
        EXPECT_FALSE(output.terminal_diagnostics.command_owned);
        EXPECT_TRUE(output.terminal_diagnostics.predicted_tail_valid);
        EXPECT_GT(output.terminal_diagnostics.stop_minimum_region_clearance_m,0.);
    }
}

TEST(TaskStop, RegionCannotDisableTheSharedStopContract) {
    SolverParams params; params.solver_backend="continuous_mpcc_acados";
    params.planning.region=stopRegion(3.).config();
    params.terminal.mpc_stop_handoff_enable=false; params.jerk_limit_enable=false;
    SpmpcProblem problem; problem.configure(params,VariantConfig{});
    EXPECT_FALSE(problem.configurationError().empty());
}

TEST(TaskStop, WideRegionAllowsTheCompleteJerkTail) {
    TaskStopManager manager;
    ASSERT_TRUE(manager.configure({}, {}, {}, kStopLimits, .6, 1.2, 1.));
    const auto region = stopRegion(3.0);
    auto input = stopInput(.0, .2, .3);
    const auto tail = manager.predict(input, &region, 0.);
    ASSERT_TRUE(tail.valid) << tail.status;
    EXPECT_TRUE(tail.region_checked);
    EXPECT_FALSE(tail.fifo_prefix_violation);
    EXPECT_GT(tail.minimum_region_clearance_m, 0.);
}

TEST(TaskStop, CandidateAfterCommandCanMakeAnOtherwiseSafeStopUnsafe) {
    TaskStopManager manager;
    ASSERT_TRUE(manager.configure({}, {}, {}, kStopLimits, .6, 1.2, 1.));
    auto input = stopInput(0., 0.);
    StopCommand candidate;
    candidate.valid = true;
    candidate.v = input.dt * input.dt;  // a = jerk_max * dt, at the limit.
    candidate.a = input.dt;
    candidate.omega = 0.;
    candidate.status = "STOP_CANDIDATE";
    const auto candidate_tail = manager.predictAfterCommand(input, candidate, nullptr, 0.);
    ASSERT_TRUE(candidate_tail.valid) << candidate_tail.status;
    ASSERT_GT(candidate_tail.distance_m, 0.);

    // Put the allowed center boundary halfway between the stationary stop and
    // the candidate tail. The current stop remains feasible while the
    // candidate's delayed command becomes region-unsafe after the FIFO.
    const auto region = stopRegion(.446 + .5 * candidate_tail.distance_m);
    const auto current_tail = manager.predict(input, &region, 0.);
    ASSERT_TRUE(current_tail.valid) << current_tail.status;
    const auto tail = manager.predictAfterCommand(input, candidate, &region, 0.);
    EXPECT_FALSE(tail.valid);
    EXPECT_FALSE(tail.fifo_prefix_violation);
    EXPECT_EQ(tail.status, "STOP_REGION_UNSAFE");
}

TEST(TaskStop, RotationQueueUsesSweptArcMotionAndFrozenCell) {
    TaskStopManager manager;
    ASSERT_TRUE(manager.configure({}, {}, {}, kStopLimits, .6, 1.2, 1.));
    const auto region = stopRegion(3.0);
    auto input = stopInput(0., .2, .3);
    const auto tail = manager.predict(input, &region, 0.);
    ASSERT_TRUE(tail.valid) << tail.status;
    EXPECT_GT(tail.distance_m, 0.);
    EXPECT_GT(tail.duration_sec, kExplicitAngularDelaySteps * input.dt);
}

TEST(TaskStop, QuietTailIncludesLinearTauResidualRegionReserve) {
    TaskStopManager manager;
    ActuatorModelParams actuator;
    actuator.linear_tau_sec = .5;
    ASSERT_TRUE(manager.configure({}, actuator, {}, kStopLimits, .6, 1.2, 1.));
    const auto region = stopRegion(1.0);
    auto input = stopInput(.5538, .001);
    input.actuator.v_cmd = .0;
    input.actuator.linear_delay_queue.fill(0.);
    const auto tail = manager.predict(input, &region, 0.);
    EXPECT_FALSE(tail.valid) << tail.status << " clearance=" << tail.minimum_region_clearance_m;
    EXPECT_EQ(tail.status, "STOP_REGION_UNSAFE") << " valid=" << tail.valid
        << " clearance=" << tail.minimum_region_clearance_m;
    EXPECT_FALSE(tail.fifo_prefix_violation);
}

TEST(TaskStop, VehicleOnlyTailAcceptsNaNLiquidWithoutFabricatingLiquidPeak) {
    TaskStopManager manager;
    ASSERT_TRUE(manager.configure({}, {}, {}, kStopLimits, .6, 1.2, 1., false));
    auto input = stopInput(0., .2);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    input.slosh = {nan, nan, nan, nan};
    const auto tail = manager.predict(input);
    ASSERT_TRUE(tail.valid) << tail.status;
    EXPECT_DOUBLE_EQ(tail.peak_height_m, 0.);
    EXPECT_DOUBLE_EQ(tail.residual_height_m, 0.);
    input.robot.v = 0.;
    input.actuator.v_cmd = 0.;
    input.actuator.linear_delay_queue.fill(0.);
    const auto readiness = manager.observe(input, true, .03, .05);
    EXPECT_TRUE(readiness.valid);

    TaskStopManager liquid_manager;
    ASSERT_TRUE(liquid_manager.configure({}, {}, {}, kStopLimits, .6, 1.2, 1., true));
    EXPECT_FALSE(liquid_manager.predict(input).valid);
}

}  // namespace spmpc_local_planner
int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
