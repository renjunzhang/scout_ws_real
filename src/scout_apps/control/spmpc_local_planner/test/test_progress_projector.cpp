#include "spmpc_local_planner/reference/progress_projector.h"
#include "spmpc_local_planner/core/spmpc_problem.h"
#include <gtest/gtest.h>

using namespace spmpc_local_planner;

namespace {

ReferencePath crossingPath() {
    ReferencePath path;
    path.setPoints({
        {0.0, 0.0, 0.0, 0.0, 0.0},
        {2.0, 2.0, 0.0, 0.0, 0.0},
        {0.0, 2.0, 0.0, 0.0, 0.0},
        {2.0, 0.0, 0.0, 0.0, 0.0}}, "map");
    return path;
}

}  // namespace

TEST(ProgressProjector, ContinuityKeepsSelfIntersectionBranch) {
    ProgressProjectionConfig config;
    config.lookahead = 2.0;
    ProgressProjector projector(config);
    ProgressProjectionState state;
    const auto path = crossingPath();

    const auto first = projector.project(path, .999, .999, state);
    ASSERT_TRUE(first.valid);
    const auto second = projector.project(path, 1.001, .999, state);
    ASSERT_TRUE(second.valid);
    const auto third = projector.project(path, .999, .999, state);
    ASSERT_TRUE(third.valid);

    EXPECT_NEAR(first.s, second.s, .02);
    EXPECT_NEAR(first.s, third.s, .02);
    EXPECT_LT(std::abs(second.s - first.s), .1);
    EXPECT_LT(std::abs(third.s - first.s), .1);
}

TEST(ProgressProjector, ProblemAndInnerSolverShareTheAcceptedBranch) {
    SolverParams params; params.solver_backend="primitive"; params.terminal.enable=false;
    VariantConfig variant; variant.slosh_enable=false; variant.w_slosh=0;
    SpmpcProblem problem; problem.configure(params,variant);
    problem.setReferencePath(crossingPath());
    SolverInput input; input.robot.x=.999; input.robot.y=.999;
    SolverOutput output;
    ASSERT_TRUE(problem.solve(input,output)) << output.status;
    const double previous=output.progress_abs_s;
    input.robot.x=1.001;
    ASSERT_TRUE(problem.solve(input,output)) << output.status;
    EXPECT_NEAR(output.progress_abs_s,previous,.01);
    // configure() is an explicit new task, even when the route is identical.
    problem.configure(params,variant); problem.setReferencePath(crossingPath());
    input.robot.x=0; input.robot.y=0;
    ASSERT_TRUE(problem.solve(input,output)) << output.status;
    EXPECT_NEAR(output.progress_abs_s,0.,1e-9);
}

TEST(ProgressProjector, OffRouteCornerCutIsNotRejectedByTrackingDistance) {
    ReferencePath route;
    route.setPoints({{0,0,0,0,0},{2,0,0,0,0},{2,2,0,0,0}},"map");
    ProgressProjector projector;
    ProgressProjectionState state;
    ASSERT_TRUE(projector.project(route,1.,0.,state).valid);
    const auto a=projector.project(route,1.4,.3,state);
    ASSERT_TRUE(a.valid); EXPECT_NEAR(a.s,1.4,1e-9);
    EXPECT_GT(a.distance,.25);
    const auto b=projector.project(route,1.7,.6,state);
    ASSERT_TRUE(b.valid); EXPECT_NEAR(b.s,2.6,1e-9);
    EXPECT_GT(b.distance,.25);
}

TEST(ProgressProjector, SelectsInteriorMinimumAfterDistanceDescent) {
    ReferencePath path;
    path.setPoints({{0, 0, 0, 0, 0}, {1, 0, 0, 0, 0},
                    {2, 0, 0, 0, 0}, {3, 0, 0, 0, 0}}, "map");
    ProgressProjector projector;
    ProgressProjectionState state;
    ASSERT_TRUE(projector.project(path, 0, 0, state).valid);
    const auto projection = projector.project(path, 1.5, 0, state);
    ASSERT_TRUE(projection.valid);
    EXPECT_NEAR(projection.s, 1.5, 1e-9);
}

TEST(ProgressProjector, FirstOfTwoLocalMinimaPreservesCurrentBranch) {
    ReferencePath path;
    path.setPoints({{0, 0, 0, 0, 0}, {1, 0, 0, 0, 0}, {2, 1, 0, 0, 0},
                    {3, 0, 0, 0, 0}, {3, 2, 0, 0, 0}, {3, 3, 0, 0, 0},
                    {4, .2, 0, 0, 0}, {5, 0, 0, 0, 0}}, "map");
    ProgressProjectionConfig config;
    config.lookahead = 10.0;
    ProgressProjector projector(config);
    ProgressProjectionState state;
    ASSERT_TRUE(projector.project(path, 0, 0, state).valid);
    const auto projection = projector.project(path, 4, 0, state);
    ASSERT_TRUE(projection.valid);
    EXPECT_NEAR(projection.point.x, 3.0, .05);
    EXPECT_LT(projection.s, 4.0);
}

TEST(ProgressProjector, NinetyDegreeCutAdvancesAcrossSegments) {
    ReferencePath path;
    path.setPoints({{0, 0, 0, 0, 0}, {1, 0, 0, 0, 0},
                    {1, 1, 0, 0, 0}, {1, 2, 0, 0, 0}}, "map");
    ProgressProjector projector;
    ProgressProjectionState state;
    ASSERT_TRUE(projector.project(path, 0, 0, state).valid);
    const auto first_turn = projector.project(path, 1, .5, state);
    ASSERT_TRUE(first_turn.valid);
    EXPECT_NEAR(first_turn.s, 1.5, 1e-9);
    const auto second_turn = projector.project(path, 1, 1.5, state);
    ASSERT_TRUE(second_turn.valid);
    EXPECT_NEAR(second_turn.s, 2.5, 1e-9);
}

TEST(ProgressProjector, NearbyFoldbackDoesNotSwitchToLaterParallelPass) {
    ReferencePath path;
    path.setPoints({{0, 0, 0, 0, 0}, {2, 0, 0, 0, 0},
                    {2, .2, 0, 0, 0}, {0, .2, 0, 0, 0},
                    {0, .4, 0, 0, 0}, {2, .4, 0, 0, 0}}, "map");
    ProgressProjectionConfig config;
    config.lookahead = 10.0;
    ProgressProjector projector(config);
    ProgressProjectionState state;
    ASSERT_TRUE(projector.project(path, 0, 0, state).valid);
    const auto projection = projector.project(path, 1, .19, state);
    ASSERT_TRUE(projection.valid);
    EXPECT_LT(projection.s, 2.0);
    EXPECT_NEAR(projection.point.y, 0.0, 1e-9);
}

TEST(ProgressProjector, ResetAllowsRelocalizationOnNewTask) {
    ProgressProjector projector;
    ProgressProjectionState state;
    const auto path = crossingPath();

    ASSERT_TRUE(projector.project(path, 1.8, .2, state).valid);
    ASSERT_GT(state.progress, 2.0);
    state.reset();
    const auto relocalized = projector.project(path, .999, .999, state);
    ASSERT_TRUE(relocalized.valid);
    EXPECT_NEAR(relocalized.s, std::sqrt(2.0), .02);
}

TEST(ProgressProjector, LookaheadIsIndependentOfControlTimestep) {
    ReferencePath path;
    path.setPoints({{0.0, 0.0, 0.0, 0.0, 0.0},
                    {10.0, 0.0, 0.0, 0.0, 0.0}}, "map");
    ProgressProjectionConfig config;
    config.lookahead = 10.0;
    ProgressProjector projector(config);
    ProgressProjectionState state;
    ASSERT_TRUE(projector.project(path, 0.0, 0.0, state).valid);
    const auto cut = projector.project(path, 5.0, 0.0, state);
    ASSERT_TRUE(cut.valid);
    EXPECT_NEAR(cut.s, 5.0, 1e-9);
}

TEST(ProgressProjector, GenericPointsKeepNonGeometricProgressCoordinate) {
    const std::vector<TrajectoryPoint> points{{0.0, 0.0, 0.0, 0.0, 0.0},
                                               {1.0, 0.0, 0.0, 0.0, 2.0},
                                               {2.0, 0.0, 0.0, 0.0, 5.0}};
    ProgressProjector projector;
    ProgressProjectionState state;
    const auto projection = projector.project(points, 1.5, .1, state, 0.0, false);
    ASSERT_TRUE(projection.valid);
    EXPECT_NEAR(projection.s, 3.5, 1e-9);
    EXPECT_NEAR(projection.point.x, 1.5, 1e-9);
}

TEST(ProgressProjector, WaitingPlateauDoesNotHideFollowingLaunch) {
    // Three coincident samples model a held position followed by a moving
    // segment.  The first cycle may legitimately land on the waiting point;
    // once the robot moves, the complete distance plateau must be skipped.
    const std::vector<TrajectoryPoint> points{{0.0, 0.0, 0.0, 0.0, 0.0},
                                               {0.0, 0.0, 0.0, 0.0, 0.0},
                                               {0.0, 0.0, 0.0, 0.0, 0.0},
                                               {2.0, 0.0, 0.0, 0.0, 2.0}};
    ProgressProjector projector;
    ProgressProjectionState state;
    ASSERT_TRUE(projector.project(points, 0.0, 0.0, state, 0.0, false).valid);
    const auto moving = projector.project(points, .5, .0, state, 0.0, false);
    ASSERT_TRUE(moving.valid);
    EXPECT_NEAR(moving.s, .5, 1e-9);
}

TEST(ProgressProjector, InvalidConfigurationDoesNotProduceProjection) {
    ProgressProjectionConfig config;
    config.lookahead = 0.0;
    ProgressProjector projector(config);
    ProgressProjectionState state;
    EXPECT_FALSE(projector.project(crossingPath(), 0.0, 0.0, state).valid);
    EXPECT_FALSE(state.initialized);
}
