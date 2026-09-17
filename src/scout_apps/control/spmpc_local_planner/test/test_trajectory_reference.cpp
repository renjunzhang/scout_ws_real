#include "spmpc_local_planner/reference/horizon_reference_builder.h"
#include "spmpc_local_planner/core/task_clock.h"
#include "spmpc_local_planner/core/spmpc_solver.h"
#include "spmpc_local_planner/planning/ocp_planning_adapter.h"
#include "spmpc_local_planner/reference/reference_spline.h"
#include "trajectory_fixture.h"
#include "../src/core/generated/ocp_parameter_contract.h"
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

TEST(TrajectoryReference, ProjectionKeepsPlanProgressInsteadOfReplacingItWithArcLength) {
    auto p=movingPlanFixture();
    for (auto& r:p.samples) {r.state[4]*=1.1; r.control[2]*=1.1;}
    p.route.back().x*=1.1; p.goal_position_tolerance+=.1;
    p.region.cells.back().s_end=p.route.back().x;
    const TrajectoryReference ref(p);
    const auto& row=p.samples[40];
    const auto projected=ref.project(row.state[0],row.state[1]);
    ASSERT_TRUE(projected.valid);
    EXPECT_NEAR(projected.s,row.state[4],1e-6);
    EXPECT_GT(std::abs(projected.s-row.state[0]),1e-4);
    EXPECT_NEAR(ref.project(0,0).s,p.samples.front().state[4],1e-9);
    const double lower=p.samples[60].state[4];
    EXPECT_GE(ref.project(row.state[0],row.state[1],lower).s,lower-1e-9);
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

TEST(TrajectoryReference, TailProgressRoundoffKeepsProjectionAndLookupInDomain) {
    auto p=movingPlanFixture();
    const double end=p.samples.back().state[4];
    p.samples[120].state[4]+=2e-11;
    p.samples[121].state[4]-=2e-11;
    const TrajectoryReference ref(p);
    const auto projected=ref.project(p.goal_pose[0]+.001,0,end+2e-11);
    ASSERT_TRUE(projected.valid);
    EXPECT_LE(projected.s,end);
    EXPECT_GE(projected.s,end-1e-12);
    EXPECT_NO_THROW(MotionRegion(p.region).clearance(p.goal_pose[0],0,projected.s));
    EXPECT_NEAR(ref.sampleAtProgress(end,4.7).t,4.7,1e-9);
    p.samples[120].state[4]+=1e-5;
    EXPECT_THROW(TrajectoryReference invalid(p),std::invalid_argument);
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

TEST(TaskClock, RejectsChangingClockSourceUntilTaskReset) {
    TaskClock clock; SolverInput input; double elapsed = -1;
    input.has_task_elapsed=true; input.task_elapsed_sec=3;
    ASSERT_TRUE(clock.observe(input,elapsed));
    input.has_task_elapsed=false; input.cycle_timing.solver_input_epoch_ns=1000000000000;
    EXPECT_FALSE(clock.observe(input,elapsed));
    EXPECT_DOUBLE_EQ(elapsed,3);
    clock.reset(); ASSERT_TRUE(clock.observe(input,elapsed));
    input.has_task_elapsed=true; input.task_elapsed_sec=100;
    EXPECT_FALSE(clock.observe(input,elapsed));
}

TEST(OcpPlanningAdapter, ChecksTerminalQueuesAndAllMotionBounds) {
    SolverParams params;
    OcpPlanningAdapter adapter(params);
    std::vector<OcpPlanningStage> stages(2);
    PredictedHorizonDebug horizon;
    horizon.states.resize(2); horizon.controls.resize(1);
    for (auto& x:horizon.states) x.model_state.assign(24,0);
    double clearance=0; std::string reason;
    ASSERT_TRUE(adapter.check(stages,horizon,clearance,reason))<<reason;
    horizon.states.back().model_state[22]=params.omega_max+.01;
    EXPECT_FALSE(adapter.check(stages,horizon,clearance,reason));
    EXPECT_EQ(reason,"MOTION_STATE_BOUND_VIOLATION");
    horizon.states.back().model_state[22]=.001;
    stages.back().task_goal_active=true;
    EXPECT_FALSE(adapter.check(stages,horizon,clearance,reason));
    horizon.states.back().model_state[22]=0;
    ASSERT_TRUE(adapter.check(stages,horizon,clearance,reason))<<reason;
    horizon.controls[0].a=params.a_max+.01;
    EXPECT_FALSE(adapter.check(stages,horizon,clearance,reason));
    EXPECT_EQ(reason,"MOTION_CONTROL_BOUND_VIOLATION");
}

TEST(OcpPlanningAdapter, RawTrackingStillGuidesTheRequiredTerminalPose) {
    using namespace ocp_parameters;
    SolverParams params;
    params.terminal.require_goal_yaw = true;
    params.terminal.mpc_stop_handoff_enable = true;
    params.terminal.goal_pose_weight = 2.;
    params.planning.geometry.enabled = false;
    params.planning.task_deadline_sec = 45.;
    OcpPlanningAdapter adapter(params);
    ReferencePath route;
    route.setPoints({{0.,0.,0.,0.,0.}, {5.,0.,.3,0.,0.}}, "map");
    SolverInput input;
    input.dt = params.actuator.dt;
    input.has_task_elapsed = true;
    input.task_elapsed_sec = 20.;
    PlanningCycleDebug debug;
    const auto stages = adapter.prepare(route, input, {0., 4., 5.}, debug);
    double p[kB0ParameterCount]{};
    adapter.write(stages.front(), p, kB0ParameterCount);
    EXPECT_DOUBLE_EQ(p[W_TASK_GOAL], 0.);
    adapter.write(stages[1], p, kB0ParameterCount);
    EXPECT_DOUBLE_EQ(p[W_TASK_GOAL], 2.);
    EXPECT_DOUBLE_EQ(p[W_CURVATURE], 0.);
    EXPECT_DOUBLE_EQ(p[W_CURVATURE_RATE], 0.);
    EXPECT_DOUBLE_EQ(p[TASK_GOAL_YAW], 0.); // Approach the position first.
    EXPECT_DOUBLE_EQ(stages[1].goal_pose[2], .3); // True final yaw is preserved.
    EXPECT_DOUBLE_EQ(p[TASK_GOAL_REQUIRE_YAW], 1.);
    EXPECT_DOUBLE_EQ(p[TASK_GOAL_ACTIVE], 0.);  // Deadline constraint is unchanged.
    input.robot.x=4.99;
    adapter.write(adapter.prepare(route,input,{5.},debug).front(),p,kB0ParameterCount);
    EXPECT_DOUBLE_EQ(p[TASK_GOAL_YAW],.3);
    input.robot.x=0.;input.task_elapsed_sec=45.;
    adapter.write(adapter.prepare(route,input,{5.},debug).front(),p,kB0ParameterCount);
    EXPECT_DOUBLE_EQ(p[TASK_GOAL_YAW],.3); // Deadline never accepts approach yaw.
    EXPECT_DOUBLE_EQ(p[TASK_GOAL_ACTIVE],1.);
    params.terminal.goal_pose_weight = -1.;
    EXPECT_THROW(OcpPlanningAdapter invalid(params), std::invalid_argument);
}

namespace {

double polynomialDerivative(const Eigen::Vector4d& c, double s) {
    return c(1) + 2.0*c(2)*s + 3.0*c(3)*s*s;
}

}  // namespace

TEST(ReferenceSpline, VerticalLineHasGeometricEndpointTangents) {
    ReferencePath path;
    path.setPoints({{0, 0, 0, 0, 0}, {0, 2, 0, 0, 0}}, "map");
    ReferenceSpline spline;
    spline.build(path);

    const auto start = spline.sample(0.0);
    const auto end = spline.sample(path.length());
    EXPECT_NEAR(start.psi, M_PI/2.0, 1e-12);
    EXPECT_NEAR(end.psi, M_PI/2.0, 1e-12);
    EXPECT_NEAR(start.kappa, 0.0, 1e-12);
    EXPECT_NEAR(end.kappa, 0.0, 1e-12);
}

TEST(ReferenceSpline, EndpointFitUsesARealSpanWithPlateauAndShortPath) {
    ReferencePath path;
    path.setPoints({{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 1e-4, 0, 0, 0}}, "map");
    ReferenceSpline spline;
    spline.build(path);
    Eigen::Vector4d cx, cy;
    fitReferencePolynomials(spline, path.length(), path.length(), cx, cy);

    const auto near_end = spline.sample(path.length()-5e-10);
    EXPECT_NEAR(near_end.psi, M_PI/2.0, 1e-12);
    EXPECT_NEAR(near_end.kappa, 0.0, 1e-12);
    const double s = path.length();
    EXPECT_NEAR(cx(0)+cx(1)*s+cx(2)*s*s+cx(3)*s*s*s, 0.0, 1e-12);
    EXPECT_NEAR(cy(0)+cy(1)*s+cy(2)*s*s+cy(3)*s*s*s, 1e-4, 1e-12);
    EXPECT_NEAR(polynomialDerivative(cx, s), 0.0, 1e-8);
    EXPECT_GT(polynomialDerivative(cy, s), 0.0);
}

TEST(ReferenceSpline, LPathFitKeepsTheTrueFinalTangent) {
    ReferencePath path;
    path.setPoints({{0, 0, 0, 0, 0}, {1, 0, 0, 0, 0}, {1, 1, 0, 0, 0}}, "map");
    ReferenceSpline spline;
    spline.build(path);
    Eigen::Vector4d cx, cy;
    fitReferencePolynomials(spline, path.length(), path.length(), cx, cy);

    const double s = path.length();
    EXPECT_NEAR(polynomialDerivative(cx, s), 0.0, 1e-8);
    EXPECT_GT(polynomialDerivative(cy, s), 0.1);
    EXPECT_NEAR(std::atan2(polynomialDerivative(cy, s), polynomialDerivative(cx, s)), M_PI/2.0, 1e-8);
    fitReferencePolynomials(spline, s-1e-8, s, cx, cy);
    EXPECT_NEAR(std::atan2(polynomialDerivative(cy, s), polynomialDerivative(cx, s)), M_PI/2.0, 1e-8);
}

TEST(ReferenceSpline, InteriorCircleKeepsCurvatureScale) {
    ReferencePath path;
    std::vector<TrajectoryPoint> points;
    for (int i=0;i<=200;++i) {
        const double a=i*.005;
        points.push_back({2.*std::cos(a),2.*std::sin(a),0,0,0});
    }
    path.setPoints(points,"map");
    ReferenceSpline spline; spline.build(path);
    EXPECT_NEAR(spline.sample(path.length()*.5).kappa,.5,.002);
}
