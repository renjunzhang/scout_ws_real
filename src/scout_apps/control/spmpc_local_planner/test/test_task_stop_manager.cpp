#include "spmpc_local_planner/core/task_stop_manager.h"
#include "spmpc_local_planner/core/spmpc_problem.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace spmpc_local_planner {

TEST(TaskStop, OrdinaryMpccDrainsCommandsWithJerkWithoutLiquidState) {
    SolverParams params;
    params.terminal.enable=true;
    params.terminal.mpc_stop_handoff_enable=true;
    params.jerk_limit_enable=true;
    params.task_stop.enable=false;
    VariantConfig variant; variant.slosh_enable=false; variant.w_slosh=0;
    SpmpcProblem problem; problem.configure(params, variant);
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
    ASSERT_TRUE(manager.configure({}, {}, {}, .6,1.2,1.));
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

TEST(TaskStop, QueueDrainAndModalVelocityGateCompletionSeparately) {
    TaskStopManager manager;TaskStopParams params;
    params.stable_hold_sec=.1;
    ASSERT_TRUE(manager.configure(params, {}, {}, .6,1.2,1.));
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
    ASSERT_TRUE(manager.configure(params, {}, {}, .6,1.2,1.));
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
    ASSERT_TRUE(manager.configure(params, {}, {}, .6,1.2,1.));
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
    ASSERT_TRUE(manager.configure(p, {}, {},.6,1.2,1.));
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
    ASSERT_TRUE(manager.configure(p, {}, {},.6,1.2,1.));
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
    ASSERT_TRUE(manager.configure(p, {}, {},.6,1.2,1.));
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

}  // namespace spmpc_local_planner
int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
