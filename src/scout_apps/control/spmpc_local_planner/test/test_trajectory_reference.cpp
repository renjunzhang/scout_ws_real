#include "spmpc_local_planner/reference/horizon_reference_builder.h"
#include "spmpc_local_planner/core/task_clock.h"
#include "trajectory_fixture.h"
#include <gtest/gtest.h>
#include <fstream>
#include <limits>

using namespace spmpc_local_planner;

TEST(TrajectoryPlan, RejectsMalformedAndIncompleteJson) {
    const std::string path="/tmp/scout_trajectory_invalid.json";
    { std::ofstream out(path); out << "{\"schema_version\":99}"; }
    EXPECT_THROW(TrajectoryPlan::load(path),std::invalid_argument);
    { std::ofstream out(path); out << "not json"; }
    EXPECT_THROW(TrajectoryPlan::load(path),std::invalid_argument);
}

TEST(TrajectoryPlan, RejectsChangedHistoryBoundsGoalAndTiming) {
    auto p=movingPlanFixture();
    p.samples[20].state[8]+=.02;
    EXPECT_THROW(p.validate(),std::invalid_argument);
    p=movingPlanFixture(); p.samples[20].control[0]=100;
    EXPECT_THROW(p.validate(),std::invalid_argument);
    p=movingPlanFixture(); p.samples.back().state[2]=1;
    EXPECT_THROW(p.validate(),std::invalid_argument);
    p=movingPlanFixture(); p.deadline+=p.dt;
    EXPECT_THROW(p.validate(),std::invalid_argument);
    p=movingPlanFixture(); p.route.front().x=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(p.validate(),std::invalid_argument);
    p=movingPlanFixture(); p.samples.back().phase="MOVE";
    EXPECT_THROW(p.validate(),std::invalid_argument);
}

TEST(TrajectoryReference, ActualSpeedAndProgressSpeedRemainDistinct) {
    auto p=movingPlanFixture(); TrajectoryReference ref(p);
    const auto t=ref.sampleAtTime(20.5*p.dt);
    EXPECT_NEAR(t.state[3],.5*(p.samples[20].state[3]+p.samples[21].state[3]),1e-12);
    EXPECT_NEAR(t.control[2],.5*(p.samples[20].control[2]+p.samples[21].control[2]),1e-12);
    EXPECT_GT(std::abs(t.state[3]-t.control[2]),1e-5);
    const double s=.5*(p.samples[20].state[4]+p.samples[21].state[4]);
    const auto spatial=ref.sampleAtProgress(s,10);
    EXPECT_NEAR(spatial.t,20.5*p.dt,1e-12);
    EXPECT_NEAR(spatial.state[3],t.state[3],1e-12);
    EXPECT_THROW(ref.sampleAtProgress(-.01,0),std::out_of_range);
    EXPECT_THROW(ref.sampleAtTime(-1),std::invalid_argument);
}

TEST(TrajectoryReference, RepeatedProgressUsesPersistentTimeWithoutRewindingTail) {
    auto p=movingPlanFixture(); TrajectoryReference ref(p);
    const double end=p.samples.back().state[4];
    EXPECT_EQ(ref.sampleAtProgress(end,4.7).phase,"TAIL");
    EXPECT_NEAR(ref.sampleAtProgress(end,4.7).t,4.7,1e-12);
    EXPECT_DOUBLE_EQ(ref.sampleAtTime(100).t,p.samples.back().t);
    // A later query cannot mutate the meaning of an earlier one.
    EXPECT_NEAR(ref.sampleAtTime(.5).t,.5,1e-12);
    EXPECT_THROW(ref.sampleAtProgress(end+.01,1),std::out_of_range);
}

TEST(HorizonReferenceBuilder, FixedTimeUsesClockAndKeepsFiniteEndpointKnots) {
    auto p=movingPlanFixture(); TrajectoryReference ref(p);
    TrajectoryReferenceConfig cfg; cfg.mode=TrajectoryReferenceMode::FixedTime;
    auto out=HorizonReferenceBuilder::build(ref,cfg,{0,p.samples.back().state[4]},1,p.dt);
    EXPECT_NEAR(out[0].v_knots[0],ref.sampleAtTime(1).state[3],1e-12);
    for (const auto& stage:out) for (int j=1;j<4;++j) EXPECT_GT(stage.s_knots[j],stage.s_knots[j-1]);
    cfg.mode=static_cast<TrajectoryReferenceMode>(50);
    EXPECT_THROW(HorizonReferenceBuilder::build(ref,cfg,{0},1,p.dt),std::invalid_argument);
}

TEST(HorizonReferenceBuilder, SpatialProfileApproximatesEveryStoredBreakpoint) {
    auto p=movingPlanFixture(); TrajectoryReference ref(p);
    TrajectoryReferenceConfig cfg; cfg.mode=TrajectoryReferenceMode::Progress;
    cfg.max_speed_error=.002; cfg.progress_window=.05;
    const auto out=HorizonReferenceBuilder::build(ref,cfg,{p.samples[40].state[4]},2,p.dt).front();
    ASSERT_EQ(out.mode,1);
    for (const auto& row:p.samples) {
        const double s=row.state[4];
        if (s<out.s_knots[0] || s>out.s_knots[3]) continue;
        const int j=s<out.s_knots[1]?0:(s<out.s_knots[2]?1:2);
        const double q=(s-out.s_knots[j])/(out.s_knots[j+1]-out.s_knots[j]);
        EXPECT_NEAR((1-q)*out.v_knots[j]+q*out.v_knots[j+1],ref.sampleAtProgress(s,2).state[3],cfg.max_speed_error);
        EXPECT_NEAR((1-q)*out.vs_knots[j]+q*out.vs_knots[j+1],ref.sampleAtProgress(s,2).control[2],cfg.max_speed_error);
    }
    auto end=HorizonReferenceBuilder::build(ref,cfg,{p.samples.back().state[4]},4.8,p.dt).front();
    EXPECT_EQ(end.mode,2); EXPECT_EQ(end.phase,"TAIL");
    EXPECT_NEAR(end.vs_knots[0],0,1e-12);
}

TEST(TaskClock, ReassemblyLatenessAndBackwardEpoch) {
    TaskClock clock; SolverInput input; double elapsed;
    input.cycle_timing.solver_input_epoch_ns=1000000000;
    ASSERT_TRUE(clock.observe(input,elapsed)); EXPECT_DOUBLE_EQ(elapsed,0);
    input.cycle_timing.solver_input_epoch_ns+=3000000000;
    ASSERT_TRUE(clock.observe(input,elapsed)); EXPECT_DOUBLE_EQ(elapsed,3);
    input.cycle_timing.solver_input_epoch_ns-=1000;
    EXPECT_FALSE(clock.observe(input,elapsed));
    clock.reset(); input.has_task_elapsed=true; input.task_elapsed_sec=5;
    ASSERT_TRUE(clock.observe(input,elapsed)); EXPECT_DOUBLE_EQ(elapsed,5);
    input.task_elapsed_sec=2; EXPECT_FALSE(clock.observe(input,elapsed));
}
