#include "spmpc_local_planner/planning/ocp_planning_adapter.h"
#include "spmpc_local_planner/core/spmpc_solver.h"
#include <iostream>

int main(int argc,char** argv) {
    if(argc!=2) { std::cerr<<"usage: validate_plan plan.json\n"; return 2; }
    try {
        const auto plan=spmpc_local_planner::TrajectoryPlan::load(argv[1]);
        spmpc_local_planner::SolverParams params;
        params.actual_v_min=plan.motion_limits[0];
        params.planning.region=plan.region;
        params.planning.evaluation_window_sec=plan.stop_window;
        params.terminal.goal_tolerance=plan.goal_position_tolerance;
        params.terminal.goal_yaw_tolerance=plan.goal_yaw_tolerance;
        params.terminal.goal_reached_max_speed=plan.stop_speed_tolerance;
        params.terminal.goal_reached_max_omega=plan.stop_omega_tolerance;
        params.planning.trajectory.mode=spmpc_local_planner::TrajectoryReferenceMode::Progress;
        params.planning.trajectory.plan_file=argv[1];
        spmpc_local_planner::OcpPlanningAdapter adapter(params);
        std::cout<<"plan, runtime physics and full-state dynamics validated: "<<plan.plan_id<<"\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<"\n"; return 1; }
}
