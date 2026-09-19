#pragma once
// Shared task/configuration and feedback conversion for native trial and replay.
#include "spmpc_local_planner/core/spmpc_problem.h"
#include <boost/property_tree/json_parser.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace spmpc_native {
using namespace spmpc_local_planner;
struct Options {
    std::string mode, prefix, plan_file;
    double actuator_scale=1., curvature_weight=.05, goal_weight=2.;
    double contour_weight=std::numeric_limits<double>::quiet_NaN();
    double progress_weight=std::numeric_limits<double>::quiet_NaN();
    int rti_iterations=5;
    int rti_min_iterations=1;
    double solve_budget_ms=0.;
};
struct TrialConfig { SolverParams params; VariantConfig variant; };

inline Options parseOptions(int argc, char** argv) {
    if (argc<4 || argc>12) throw std::invalid_argument(
        "usage: geometry_trial raw|geometry|planned|planned_slosh|external_timed output-prefix plan.json "
        "[actuator-scale] [curvature-weight] [contour-weight] [rti-iterations] [goal-weight] [progress-weight] "
        "[solve-budget-ms] [rti-min-iterations]");
    Options out;
    out.mode=argv[1]; out.prefix=argv[2]; out.plan_file=argv[3];
    if (out.mode!="raw" && out.mode!="geometry" && out.mode!="planned" && out.mode!="planned_slosh" && out.mode!="external_timed")
        throw std::invalid_argument("unknown trial mode");
    if(argc>4) out.actuator_scale=std::stod(argv[4]);
    if(argc>5) out.curvature_weight=std::stod(argv[5]);
    if(argc>6) out.contour_weight=std::stod(argv[6]);
    if(argc>7) out.rti_iterations=std::stoi(argv[7]);
    if(argc>8) out.goal_weight=std::stod(argv[8]);
    if(argc>9) out.progress_weight=std::stod(argv[9]);
    if(argc>10) out.solve_budget_ms=std::stod(argv[10]);
    if(argc>11) out.rti_min_iterations=std::stoi(argv[11]);
    if (!std::isfinite(out.solve_budget_ms) || out.solve_budget_ms<0. ||
        out.rti_min_iterations<1 || out.rti_min_iterations>out.rti_iterations)
        throw std::invalid_argument("invalid solve budget/minimum RTI count");
    if (!std::isfinite(out.actuator_scale) || out.actuator_scale<=0 ||
        !std::isfinite(out.curvature_weight) || out.curvature_weight<0 ||
        !std::isfinite(out.goal_weight) || out.goal_weight<0 ||
        (argc>6 && (!std::isfinite(out.contour_weight) || out.contour_weight<0)))
        throw std::invalid_argument("invalid trial scale/weight");
    if (argc>9 && (out.mode!="planned_slosh" || !std::isfinite(out.progress_weight) || out.progress_weight<0))
        throw std::invalid_argument("progress-weight requires planned_slosh and a finite nonnegative value");
    return out;
}

inline TrialConfig makeConfig(const Options& options, const TrajectoryPlan& plan) {
    const auto& mode=options.mode;
    TrialConfig config;
    auto& params=config.params;
    params.rti_iterations=options.rti_iterations;
    params.rti_min_iterations=options.rti_min_iterations;
    params.max_prediction_defect=1e-4;
    params.actual_v_min=-.002; params.jerk_limit_enable=true;
    params.warm_start.enable=true; params.warm_start.type="diff_drive_flatness";
    params.warm_start.fallback_to_previous_solution=false;
    params.warm_start.fallback_to_primitive=true;
    params.platform.kinematics="differential";
    params.terminal.mpc_stop_handoff_enable=true;
    params.terminal.goal_tolerance=.05;
    params.terminal.goal_reached_max_speed=.01;
    params.terminal.goal_reached_max_omega=.02;
    params.planning.experiment_profile_id=mode;
    params.planning.liquid_free_baseline=mode=="raw";
    params.terminal.require_goal_yaw=true;
    params.terminal.goal_pose_weight=2.;
    params.planning.geometry.enabled=mode!="raw";
    params.planning.geometry.curvature_weight=mode=="raw"?0.:options.curvature_weight;
    params.planning.geometry.curvature_rate_weight=mode=="raw"?0.:.01;
    params.planning.geometry.goal_weight=mode=="raw" ? 0 : options.goal_weight;
    params.planning.geometry.reference_curvature_speed_limit=mode=="raw";
    auto& variant=config.variant;
    variant.name=mode=="planned_slosh"?"B_slosh":"B0";
    variant.slosh_enable=mode=="planned_slosh";
    variant.slosh_constraint_enable=false;
    variant.w_slosh=variant.slosh_enable?5.:0.;
    variant.w_contour=std::isfinite(options.contour_weight)?options.contour_weight:(mode=="raw"?1.:.02);
    variant.w_lag=.2; variant.w_progress=.2; variant.w_v=1.;variant.w_vs=.3;
    variant.w_control=.1;variant.w_accel=0;variant.w_alpha=.1;variant.w_du_a=.1;variant.w_du_vs=.1;
    variant.v_ref=.25;
    params.planning.region=plan.region;
    params.planning.task_deadline_sec=plan.deadline;
    params.planning.evaluation_window_sec=plan.stop_window;
    params.terminal.goal_tolerance=plan.goal_position_tolerance;
    params.terminal.goal_yaw_tolerance=plan.goal_yaw_tolerance;
    params.terminal.goal_reached_max_speed=plan.stop_speed_tolerance;
    params.terminal.goal_reached_max_omega=plan.stop_omega_tolerance;
    params.actual_v_min=plan.motion_limits[0]; params.v_max=plan.motion_limits[1];
    params.omega_max=plan.motion_limits[2]; params.a_max=plan.motion_limits[3];
    params.actual_jerk_max=plan.actual_jerk_max;
    params.alpha_max=plan.motion_limits[4]; params.jerk_max=plan.motion_limits[5];
    params.actuator.linear_tau_sec=plan.actuator_parameters[0];
    params.actuator.angular_tau_sec=plan.actuator_parameters[1];
    params.actuator.linear_gain=plan.actuator_parameters[2];
    params.actuator.angular_gain=plan.actuator_parameters[3];
    if (mode=="planned" || mode=="planned_slosh") {
        params.planning.trajectory.mode=TrajectoryReferenceMode::Progress;
        params.planning.trajectory.plan_file=options.plan_file;
    }
    if (mode=="planned_slosh" && std::isfinite(options.progress_weight)) {
        // Explicit work-point trial: match the current ROS planned_slosh
        // profile while preserving defaults for historical native invocations.
        params.qp_iteration_limit=20;
        params.planning.geometry.curvature_rate_weight=.0001;
        variant.w_progress=options.progress_weight;
    }
    if (mode=="external_timed") {
        params.qp_iteration_limit=20;
        params.planning.trajectory.mode=TrajectoryReferenceMode::TimeTracking;
        params.planning.trajectory.plan_file=options.plan_file;
        params.planning.geometry.curvature_weight=0.;
        params.planning.geometry.curvature_rate_weight=0.;
        params.planning.geometry.goal_weight=2.;
        variant.w_contour=1.; variant.w_progress=0.; variant.w_v=10.;
    }
    return config;
}

inline SolverInput makeInput(const std::vector<double>& state, double time) {
    SolverInput input;
    input.robot={state[0],state[1],state[2],state[3],state[5]};
    input.slosh.eta_x=state[24]; input.slosh.eta_x_dot=state[25];
    input.slosh.eta_y=state[26]; input.slosh.eta_y_dot=state[27];
    input.actuator.valid=true; input.actuator.v_cmd=state[6];
    input.actuator.omega_cmd=state[7]; input.actuator.a_cmd_memory=state[23];
    std::copy_n(state.begin()+8,5,input.actuator.linear_delay_queue.begin());
    std::copy_n(state.begin()+13,10,input.actuator.angular_delay_queue.begin());
    input.has_task_elapsed=true; input.task_elapsed_sec=time;
    return input;
}

inline std::vector<double> taskInitialState(const std::string& file, const TrajectoryPlan& plan) {
    boost::property_tree::ptree data;
    boost::property_tree::read_json(file,data);
    std::vector<double> state;
    for (const auto& row:data.get_child("task.start_state")) state.push_back(row.second.get_value<double>());
    if (state.size()!=28) throw std::invalid_argument("task initial state must have 28 entries");
    for (size_t i=0;i<state.size();++i)
        if (!std::isfinite(state[i]) || std::abs(state[i]-plan.samples.front().state[i])>2e-6)
            throw std::invalid_argument("plan initial state differs from the declared plant initial state");
    // The plant starts at the specified task state. An optimizer's numerical
    // residual at x[0] is not a real previously published negative command.
    return state;
}

}  // namespace spmpc_native
