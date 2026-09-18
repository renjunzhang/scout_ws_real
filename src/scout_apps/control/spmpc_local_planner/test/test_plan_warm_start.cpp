#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"
#include "trajectory_fixture.h"
#include <boost/property_tree/json_parser.hpp>
#include <gtest/gtest.h>
#include <cstdio>
#include <unistd.h>

#ifdef SPMPC_TEST_WITH_SLOSH
namespace spmpc_local_planner {
namespace {
using boost::property_tree::ptree;

template<class Values> ptree jsonArray(const Values& values) {
    ptree result;
    for (const auto value : values) {
        ptree item; item.put_value(value); result.push_back({"", item});
    }
    return result;
}

// Serialize the small dynamics-generated fixture, avoiding a frozen bulky plan.
class PlanWarmStart : public ::testing::Test {
protected:
    TrajectoryPlan plan = movingPlanFixture();
    SolverParams params;
    VariantConfig variant;
    ReferencePath route;
    ContinuousMpccSolverAcados solver;
    std::string path;

    void SetUp() override {
        char name[] = "/tmp/scout_plan_seed_XXXXXX";
        const int fd = mkstemp(name);
        ASSERT_GE(fd, 0); close(fd); path = name;
        params.planning.region = plan.region;
        params.planning.trajectory.mode = TrajectoryReferenceMode::Progress;
        params.planning.trajectory.plan_file = path;
        params.planning.evaluation_window_sec = plan.stop_window;
        params.actual_v_min = plan.motion_limits[0];
        params.terminal.goal_tolerance = plan.goal_position_tolerance;
        params.terminal.goal_yaw_tolerance = plan.goal_yaw_tolerance;
        params.terminal.goal_reached_max_speed = plan.stop_speed_tolerance;
        params.terminal.goal_reached_max_omega = plan.stop_omega_tolerance;
        params.rti_iterations = 5; params.max_prediction_defect = 1e-4;
        params.warm_start.enable = true;
        params.warm_start.use_previous_solution = true;
        variant.name = "B_slosh"; variant.slosh_enable = true; variant.w_slosh = 5.;
        variant.w_alpha = variant.w_du_a = variant.w_du_vs = .1;
        ptree data, planning, rows, points;
        std::istringstream encoded(planningConfigJson(params.planning));
        boost::property_tree::read_json(encoded, planning);
        for (auto& cell : planning.get_child("region.cells")) {
            ptree vertices;
            for (const auto& xy : cell.second.get_child("vertices"))
                vertices.push_back({"", jsonArray(std::array<double,2>{{
                    xy.second.get<double>("x"), xy.second.get<double>("y")}})});
            cell.second.put_child("vertices", vertices);
        }
        data.add_child("region", planning.get_child("region"));
        data.put("schema_version", plan.schema_version);
        data.put("liquid_model_version", plan.liquid_model_version);
        data.put("cost_model_version", plan.cost_model_version);
        data.put("plan_id", plan.plan_id); data.put("frame_id", plan.frame_id);
        data.put("region_id", plan.region_id); data.put("dt", plan.dt);
        data.put("transport_duration", plan.transport_duration);
        data.put("deadline", plan.deadline); data.put("stop_window", plan.stop_window);
        data.put("height_coeff", plan.height_coeff);
        data.put("goal_position_tolerance", plan.goal_position_tolerance);
        data.put("goal_yaw_tolerance", plan.goal_yaw_tolerance);
        data.put("stop_speed_tolerance", plan.stop_speed_tolerance);
        data.put("stop_omega_tolerance", plan.stop_omega_tolerance);
        data.add_child("actuator_parameters", jsonArray(plan.actuator_parameters));
        data.add_child("liquid_parameters", jsonArray(plan.liquid_parameters));
        data.add_child("goal_pose", jsonArray(plan.goal_pose));
        const char* limits[] = {"actual_v_min", "v_max", "omega_max", "a_max", "alpha_max", "jerk_max"};
        for (size_t i=0; i<6; ++i) data.put(std::string("motion_limits.")+limits[i], plan.motion_limits[i]);
        std::vector<TrajectoryPoint> route_points;
        for (const auto& xy : plan.route) {
            points.push_back({"", jsonArray(std::array<double,2>{{xy.x,xy.y}})});
            route_points.push_back({xy.x,xy.y,0.,0.,0.});
        }
        route.setPoints(route_points, plan.frame_id);
        for (const auto& sample : plan.samples) {
            ptree row; row.put("t", sample.t); row.put("phase", sample.phase);
            row.add_child("state", jsonArray(sample.state));
            row.add_child("control", jsonArray(sample.control)); rows.push_back({"", row});
        }
        data.add_child("route", points); data.add_child("samples", rows);
        boost::property_tree::write_json(path, data);
        solver.configure(params, variant);
    }
    void TearDown() override { if (!path.empty()) std::remove(path.c_str()); }

    SolverInput input() const {
        SolverInput result; result.has_task_elapsed = true; result.actuator.valid = true;
        return result;
    }
};

TEST_F(PlanWarmStart, PreparationKeepsLiveHistoryEmptyAndRequiresOnlineRti) {
    solver.prepareReference(route);  // A route notification must retain the prepared plan.
    auto observed = input();
    observed.solve_budget.deadline = SolveBudget::Clock::now()+std::chrono::seconds(1);
    SolverOutput output;
    ASSERT_TRUE(solver.solve(observed, route, output)) << output.status;
    EXPECT_EQ(output.pre_solve_snapshot.warm_start_source, "PREPARED_TRAJECTORY_PLAN");
    EXPECT_FALSE(output.pre_solve_snapshot.have_previous_solution);
    EXPECT_FALSE(output.pre_solve_snapshot.have_previous_control);
    EXPECT_DOUBLE_EQ(output.pre_solve_snapshot.previous_a, 0.);
    EXPECT_DOUBLE_EQ(output.wall_timing.iteration_estimate_ms, 0.);
    EXPECT_GT(output.pre_solve_snapshot.rti_iterations, 0);
    EXPECT_LE(output.predicted_horizon.dynamics_max_defect, params.max_prediction_defect);
    observed.solve_budget = {};
    ASSERT_TRUE(solver.solve(observed, route, output)) << output.status;
    EXPECT_EQ(output.pre_solve_snapshot.warm_start_source, "SHIFTED_PREVIOUS_SOLUTION");
    EXPECT_TRUE(output.pre_solve_snapshot.have_previous_solution);
}

TEST_F(PlanWarmStart, ExpiredBudgetPreservesSeedAndFeedbackReplacesEveryNominalState) {
    auto observed = input();
    observed.robot = {.003,.002,.001,.01,.002};
    observed.slosh = {1e-6,2e-6,3e-6,4e-6};
    observed.actuator.v_cmd = .013; observed.actuator.omega_cmd = .005;
    observed.actuator.a_cmd_memory = .012;
    for (size_t k=0; k<observed.actuator.linear_delay_queue.size(); ++k)
        observed.actuator.linear_delay_queue[k] = .004+.001*k;
    for (size_t k=0; k<observed.actuator.angular_delay_queue.size(); ++k)
        observed.actuator.angular_delay_queue[k] = .001+.0001*k;
    observed.solve_budget.deadline = SolveBudget::Clock::now()-std::chrono::seconds(1);
    SloshDynamics liquid; ASSERT_TRUE(liquid.configure(params.slosh));
    for (int attempt=0; attempt<2; ++attempt) {
        SolverOutput output;
        EXPECT_FALSE(solver.solve(observed, route, output));
        EXPECT_EQ(output.status, "SOLVE_BUDGET_EXHAUSTED");
        const auto& snap = output.pre_solve_snapshot;
        EXPECT_EQ(snap.rti_iterations, 0);
        EXPECT_EQ(snap.warm_start_source, "PREPARED_TRAJECTORY_PLAN");
        EXPECT_FALSE(snap.have_previous_solution); EXPECT_FALSE(snap.have_previous_control);
        std::vector<double> state{.003,.002,.001,.01,snap.s0,.002,.013,.005};
        state.insert(state.end(), observed.actuator.linear_delay_queue.begin(), observed.actuator.linear_delay_queue.end());
        state.insert(state.end(), observed.actuator.angular_delay_queue.begin(), observed.actuator.angular_delay_queue.end());
        state.insert(state.end(), {.012,1e-6,2e-6,3e-6,4e-6});
        ASSERT_EQ(snap.initial_guess_states.size(), static_cast<size_t>(snap.horizon_steps+1));
        for (int k=0; k<=snap.horizon_steps; ++k) {
            for (size_t j=0; j<state.size(); ++j)
                EXPECT_NEAR(snap.initial_guess_states[k].model_state[j], state[j], 1e-12);
            if (k==snap.horizon_steps) break;
            const auto& control = snap.initial_guess_controls[k];
            const std::array<double,3> u{{control.a, control.alpha_or_omega, control.v_s}};
            std::vector<double> next;
            ASSERT_TRUE(stepExplicitState(state, u, params.actuator, liquid, observed.dt, next));
            state = std::move(next);
        }
    }
}

TEST_F(PlanWarmStart, LateStartAndReconfiguredRawBaselineDoNotUsePreparedSeed) {
    auto observed = input(); observed.task_elapsed_sec = 2*observed.dt;
    observed.solve_budget.deadline = SolveBudget::Clock::now()-std::chrono::seconds(1);
    SolverOutput output;
    EXPECT_FALSE(solver.solve(observed, route, output));
    EXPECT_EQ(output.pre_solve_snapshot.warm_start_source, "TRAJECTORY_PLAN");
    params.planning.trajectory = {};
    params.planning.liquid_free_baseline = true;
    variant.name = "B0"; variant.slosh_enable = false; variant.w_slosh = 0.;
    solver.configure(params, variant);
    EXPECT_FALSE(solver.solve(observed, route, output));
    EXPECT_EQ(output.status, "SOLVE_BUDGET_EXHAUSTED");
    EXPECT_NE(output.pre_solve_snapshot.warm_start_source, "PREPARED_TRAJECTORY_PLAN");
    EXPECT_FALSE(output.pre_solve_snapshot.have_previous_solution);
    EXPECT_FALSE(output.pre_solve_snapshot.have_previous_control);
}

TEST_F(PlanWarmStart, RawReferencePreparationPreservesBudgetAndReanchorsFeedback) {
    params.planning.trajectory = {};
    params.planning.liquid_free_baseline = true;
    variant.name = "B0"; variant.slosh_enable = false; variant.w_slosh = 0.0;
    solver.configure(params, variant);
    solver.prepareReference(route);
    auto observed = input();
    observed.robot = {.003, .002, .001, .01, .002};
    observed.actuator.v_cmd = .013;
    observed.actuator.a_cmd_memory = .012;
    observed.actuator.linear_delay_queue.fill(.004);
    observed.solve_budget.deadline = SolveBudget::Clock::now() - std::chrono::seconds(1);
    for (int attempt = 0; attempt < 2; ++attempt) {
        SolverOutput output;
        EXPECT_FALSE(solver.solve(observed, route, output));
        EXPECT_EQ(output.status, "SOLVE_BUDGET_EXHAUSTED");
        const auto& snapshot = output.pre_solve_snapshot;
        EXPECT_EQ(snapshot.warm_start_source, "PREPARED_REFERENCE_PATH");
        EXPECT_FALSE(snapshot.have_previous_solution);
        EXPECT_FALSE(snapshot.have_previous_control);
        EXPECT_EQ(snapshot.rti_iterations, 0);
        const auto& initial = snapshot.initial_guess_states.front();
        EXPECT_DOUBLE_EQ(initial.model_state[0], observed.robot.x);
        EXPECT_DOUBLE_EQ(initial.model_state[3], observed.robot.v);
        EXPECT_DOUBLE_EQ(initial.model_state[6], observed.actuator.v_cmd);
        EXPECT_DOUBLE_EQ(initial.model_state[8], observed.actuator.linear_delay_queue.front());
        EXPECT_DOUBLE_EQ(initial.model_state[23], observed.actuator.a_cmd_memory);
    }
}

}  // namespace
}  // namespace spmpc_local_planner
#endif
