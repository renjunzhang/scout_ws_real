#include "spmpc_local_planner/dynamics/actual_jerk.h"
#include "spmpc_local_planner/dynamics/explicit_state_rollout.h"
#include "spmpc_local_planner/core/task_stop_manager.h"
#include <gtest/gtest.h>
#include <cmath>

namespace spmpc_local_planner {
TEST(ActualJerk, RowMatchesProductionAcrossResponseParametersAndBraking) {
    SloshDynamics liquid;
    ASSERT_TRUE(liquid.configure(SloshModelParams{}));
    for (double tau : {.07,.112,.25}) for (double gain : {.85,1.018,1.2}) {
        ActuatorModelParams model; model.linear_tau_sec=tau; model.linear_gain=gain;
        for (double sign : {-1.,1.}) {
            std::vector<double> x(28,0.), next;
            x[3]=.25; x[8]=.25/gain; x[9]=x[8]+sign*.01;
            // Held-input derivative is zero, but FIFO switch produces jerk.
            ASSERT_TRUE(stepExplicitState(x,{{0.,0.,0.}},model,liquid,model.dt,next));
            const double delta=actualAcceleration(next[3],next[8],model)-actualAcceleration(x[3],x[8],model);
            EXPECT_NEAR(actualAccelerationDelta(x[3],x[8],x[9],model,model.dt),delta,1e-12);
            EXPECT_GT(std::abs(delta/model.dt),1.);
            x[3]=.37;
            ASSERT_TRUE(stepExplicitState(x,{{.1,0.,0.}},model,liquid,model.dt,next));
            EXPECT_NEAR(actualAccelerationDelta(x[3],x[8],x[9],model,model.dt),
                actualAcceleration(next[3],next[8],model)-actualAcceleration(x[3],x[8],model),1e-12);
        }
    }
}
TEST(ActualJerk, StopTailRejectsImmutableHistoryAndAllowsDisabledContract) {
    TaskStopManager manager;
    ActuatorModelParams model;
    SolverInput input; input.dt=model.dt; input.actuator.valid=true;
    input.actuator.v_cmd=.1; input.actuator.linear_delay_queue.fill(.1);
    input.robot.v=model.linear_gain*.1;
    input.actuator.linear_delay_queue[1]=.11;
    ASSERT_TRUE(manager.configure(TaskStopParams{},model,SloshModelParams{}, {-.002,.8,1.2},.6,1.2,1.,false,1.));
    auto tail=manager.predict(input);
    EXPECT_FALSE(tail.valid); EXPECT_EQ(tail.status,"STOP_ACTUAL_JERK_VIOLATION"); EXPECT_TRUE(tail.fifo_prefix_violation);
    ASSERT_TRUE(manager.configure(TaskStopParams{},model,SloshModelParams{}, {-.002,.8,1.2},.6,1.2,1.,false,0.));
    EXPECT_TRUE(manager.predict(input).valid);
    // An ordinary moving stop must be generated under the actual bound,
    // not merely rejected because the old command-only brake overshoots it.
    input.actuator.linear_delay_queue.fill(.1);
    ASSERT_TRUE(manager.configure(TaskStopParams{},model,SloshModelParams{}, {-.002,.8,1.2},.6,1.2,1.,false,1.));
    EXPECT_TRUE(manager.predict(input).valid);
    input=SolverInput{};input.dt=model.dt;input.actuator.valid=true;
    ASSERT_TRUE(manager.configure(TaskStopParams{},model,SloshModelParams{}, {-.002,.8,1.2},.6,1.2,1.,false,1.));
    EXPECT_TRUE(manager.predict(input).valid);
}
} // namespace spmpc_local_planner
