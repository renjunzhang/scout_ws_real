// Deterministic model-in-the-loop trial. It deliberately has no ROS/robot I/O.
#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/dynamics/explicit_state_rollout.h"
#include "spmpc_local_planner/dynamics/actual_jerk.h"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

using namespace spmpc_local_planner;

namespace {
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

Options parseOptions(int argc, char** argv) {
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

TrialConfig makeConfig(const Options& options, const TrajectoryPlan& plan) {
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

SolverInput makeInput(const std::vector<double>& state, double time) {
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

std::vector<double> taskInitialState(const std::string& file, const TrajectoryPlan& plan) {
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

double percentile(std::vector<double> values, double q) {
    if(values.empty()) return 0.0;
    std::sort(values.begin(),values.end());
    const size_t i=std::min(values.size()-1,static_cast<size_t>(std::ceil(q*values.size())-1));
    return values[i];
}

boost::property_tree::ptree parameterManifest(const SolverParams& p, const VariantConfig& v,
    const TrajectoryPlan& plan, const PreSolveSnapshotDebug& snapshot) {
    boost::property_tree::ptree result, planning;
    std::istringstream encoded(planningConfigJson(p.planning));
    boost::property_tree::read_json(encoded,planning);
    result.add_child("planning",planning);
    result.put("source","native explicit configuration; not a ROS launch validation");
    result.put("variant",v.name);
    result.put("rti_iterations",p.rti_iterations);
    result.put("rti_min_iterations",p.rti_min_iterations);
    result.put("prediction_defect_limit",p.max_prediction_defect);
    result.put("jerk_limit_enable",p.jerk_limit_enable);
    result.put("actual_jerk_max",p.actual_jerk_max);
    result.put("terminal_goal_position_tolerance",p.terminal.goal_tolerance);
    result.put("terminal_goal_yaw_tolerance",p.terminal.goal_yaw_tolerance);
    result.put("terminal_stop_speed",p.terminal.goal_reached_max_speed);
    result.put("terminal_stop_omega",p.terminal.goal_reached_max_omega);
    result.put("liquid_constraint_enable",v.slosh_constraint_enable);
    result.put("liquid_complete_stop",p.task_stop.enable);
    result.put("terminal_goal_pose_weight",p.terminal.goal_pose_weight);
    result.put("use_previous_solution",p.warm_start.use_previous_solution);
    result.put("height_coeff",plan.height_coeff);
    const char* names[]={"actual_v_min","v_max","omega_max","a_max","alpha_max","jerk_max"};
    for(size_t i=0;i<6;++i) result.put(std::string("motion_limits.")+names[i],plan.motion_limits[i]);
    const size_t width=snapshot.parameter_names.size();
    if (width && snapshot.stage_parameters.size()>=width)
        for(size_t i=0;i<width;++i)
            result.put("first_ocp_stage."+snapshot.parameter_names[i],snapshot.stage_parameters[i]);
    return result;
}

int runTrial(const Options& options) {
    const auto& mode=options.mode;
    const auto& prefix=options.prefix;
    const auto plan=TrajectoryPlan::load(options.plan_file);
    auto config=makeConfig(options,plan);
    auto& params=config.params;
    auto& variant=config.variant;
    std::vector<TrajectoryPoint> points;
    std::vector<double> state=taskInitialState(options.plan_file,plan);
    const double end_time=plan.deadline+plan.stop_window;
    for(size_t i=0;i<plan.route.size();++i) {
        const auto& xy=plan.route[i];
        const double yaw=i+1<plan.route.size() ?
            std::atan2(plan.route[i+1].y-xy.y,plan.route[i+1].x-xy.x) : plan.goal_pose[2];
        points.push_back({xy.x,xy.y,yaw,0,0});
    }
    ReferencePath route; route.setPoints(points,plan.frame_id);
    SpmpcProblem problem; problem.configure(params,variant);problem.setReferencePath(route);
    if(!problem.configurationError().empty()) {
        std::cerr<<problem.configurationError()<<'\n'; return 2;
    }
    SloshDynamics liquid;
    if (!liquid.configure(params.slosh)) { std::cerr<<"invalid liquid model\n"; return 2; }
    const auto liquid_coeffs=liquid.coefficients().values();
    for (size_t i=0;i<liquid_coeffs.size();++i)
        if (std::abs(liquid_coeffs[i]-plan.liquid_parameters[i])>1e-9) {
            std::cerr<<"liquid parameter mismatch at "<<i<<"\n"; return 2;
        }
    if (std::abs(liquid.heightCoeff()-plan.height_coeff)>1e-9) throw std::invalid_argument("height coefficient mismatch");
    if (std::abs(params.actuator.dt-plan.dt)>1e-9) throw std::invalid_argument("model dt mismatch");
    ActuatorModelParams plant=params.actuator;
    const double actuator_scale=options.actuator_scale;
    plant.linear_tau_sec*=actuator_scale;plant.angular_tau_sec*=actuator_scale;
    std::ofstream csv(prefix+".csv");
    if (!csv) throw std::runtime_error("cannot open trial CSV");
    csv<<std::setprecision(17);
    csv<<"t,x,y,yaw,v,omega,v_cmd,omega_cmd,a_cmd,alpha_cmd,vs,height,status,projected_s,ref_x,ref_y,ref_yaw\n";
    double curvature_energy=0, distance=0, peak=0, minimum_clearance=1e100, completion_time=-1;
    double max_jerk=0, max_actual_jerk=0, max_solve_ms=0, max_defect=0;
    double min_v=std::numeric_limits<double>::infinity(), max_v=-min_v;
    double min_omega=std::numeric_limits<double>::infinity(), max_omega=-min_omega;
    std::vector<double> wall_ms, solver_ms;
    std::vector<double> cycle_ms;
    std::ofstream timing(prefix+"_timing.csv");
    if (!timing) throw std::runtime_error("cannot open timing CSV");
    timing<<std::setprecision(17)<<"t,iterations,cycle_ms,solve_ms,setup_ms,rti_ms,residual_ms,status\n";
    int spins=0,failures=0;std::string failure_status;
    MotionRegion region(params.planning.region);
    PreSolveSnapshotDebug first_snapshot;
    double last_time=0.;
    const int last_k=static_cast<int>(std::floor(end_time/plant.dt+1e-9));
    for(int k=0;k<=last_k;++k) {
        const double t=k*plant.dt;
        const auto cycle_begin=SolveBudget::Clock::now();
        auto input=makeInput(state,t);
        if (options.solve_budget_ms>0.)
            input.solve_budget.deadline=cycle_begin+std::chrono::duration_cast<SolveBudget::Clock::duration>(
                std::chrono::duration<double,std::milli>(options.solve_budget_ms));
        last_time=t;
        SolverOutput out;
        const auto solve_begin=std::chrono::steady_clock::now();
        const bool ok=problem.solve(input,out);
        if (first_snapshot.parameter_names.empty() && !out.pre_solve_snapshot.parameter_names.empty())
            first_snapshot=out.pre_solve_snapshot;
        const auto solve_end=std::chrono::steady_clock::now();
        wall_ms.push_back(std::chrono::duration<double,std::milli>(solve_end-solve_begin).count());
        cycle_ms.push_back(std::chrono::duration<double,std::milli>(solve_end-cycle_begin).count());
        timing<<t<<','<<out.wall_timing.iterations<<','<<cycle_ms.back()<<','<<wall_ms.back()<<','
              <<out.wall_timing.setup_ms<<','<<out.wall_timing.rti_ms<<','<<out.wall_timing.residual_ms
              <<','<<out.status<<'\n';
        if (options.solve_budget_ms>0. && cycle_ms.back()>options.solve_budget_ms) {
            ++failures; failure_status="NATIVE_SOLVE_DEADLINE_MISSED";
        }
        solver_ms.push_back(out.solver_time_ms);
        max_solve_ms=std::max(max_solve_ms,out.solver_time_ms);
        max_defect=std::max(max_defect,out.predicted_horizon.dynamics_max_defect);
        if(!ok || !out.success) {
            ++failures; failure_status=out.status;
            const auto& snap=out.pre_solve_snapshot;
            std::ofstream failed(prefix+"_failed_horizon.csv");
            failed<<"k,x,y,s,reference_s0,reference_s3\n";
            const auto index=[&](const std::string& name) {
                return std::distance(snap.parameter_names.begin(),std::find(snap.parameter_names.begin(),snap.parameter_names.end(),name));
            };
            const size_t width=snap.parameter_names.size(), left=index("reference_s0"),right=index("reference_s3");
            for(size_t j=0;j<out.predicted_horizon.states.size();++j) {
                const auto& z=out.predicted_horizon.states[j];
                failed<<j<<','<<z.x<<','<<z.y<<','<<z.s;
                if(left<width&&right<width&&(j+1)*width<=snap.stage_parameters.size())
                    failed<<','<<snap.stage_parameters[j*width+left]<<','<<snap.stage_parameters[j*width+right];
                failed<<'\n';
            }
        }
        if (k==30 && out.predicted_horizon.valid && out.predicted_horizon.states.back().model_state.size()==28) {
            std::ofstream checkpoint(prefix+"_checkpoint.json");
            checkpoint<<std::setprecision(17)<<"{\"source\":\"actual solver predicted terminal state\",\"task_elapsed_sec\":"
                <<t+out.predicted_horizon.controls.size()*plant.dt<<",\"state\":[";
            const auto& predicted=out.predicted_horizon.states.back().model_state;
            for (size_t j=0;j<predicted.size();++j) checkpoint<<(j?",":"")<<predicted[j];
            checkpoint<<"]}\n";
        }
        if(out.status=="GOAL_REACHED"&&completion_time<0) completion_time=t;
        const double vcommand=ok?out.cmd_v:0, wcommand=ok?out.cmd_omega:0;
        std::array<double,3> control{{(vcommand-state[6])/plant.dt,(wcommand-state[7])/plant.dt,0}};
        if(out.predicted_horizon.valid&&!out.predicted_horizon.controls.empty()) control[2]=out.predicted_horizon.controls.front().v_s;
        min_v=std::min(min_v,state[3]); max_v=std::max(max_v,state[3]);
        min_omega=std::min(min_omega,state[5]); max_omega=std::max(max_omega,state[5]);
        if(ok && out.success) {
            max_jerk=std::max(max_jerk,std::abs(control[0]-state[23])/plant.dt);
            if(vcommand < -1e-6 || vcommand>params.v_max+1e-6 || std::abs(wcommand)>params.omega_max+1e-6 ||
               std::abs(control[0])>params.a_max+1e-6 || std::abs(control[1])>params.alpha_max+1e-6 ||
               max_jerk>params.jerk_max+1e-4) {
                ++failures;failure_status="PLANT_COMMAND_BOUND_VIOLATION";
            }
        }
        if (state[3]<params.actual_v_min-1e-6 || state[3]>params.v_max+1e-6 ||
            std::abs(state[5])>params.omega_max+1e-6) {
            ++failures; failure_status="PLANT_ACTUAL_MOTION_BOUND_VIOLATION";
        }
        const double height=liquid.height(input.slosh), speed=std::sqrt(state[3]*state[3]+.05*.05);
        peak=std::max(peak,height);
        if (t < end_time-1e-9) { curvature_energy+=state[5]*state[5]/speed*plant.dt; distance+=std::abs(state[3])*plant.dt; }
        if(std::abs(state[3])<.03&&std::abs(state[5])>.1) ++spins;
        const double route_s=std::max(0.,std::min(route.length(),out.progress_abs_s));
        minimum_clearance=std::min(minimum_clearance,region.clearance(state[0],state[1],route_s));
        csv<<t<<','<<state[0]<<','<<state[1]<<','<<state[2]<<','<<state[3]<<','<<state[5]<<','<<state[6]<<','<<state[7]<<','<<control[0]<<','<<control[1]<<','<<control[2]<<','<<height<<','<<out.status<<','<<out.progress_abs_s<<','<<out.stage0_reference_debug.ref_x<<','<<out.stage0_reference_debug.ref_y<<','<<out.stage0_reference_debug.ref_yaw<<'\n';
        if(minimum_clearance < -1e-6) {++failures;failure_status="PLANT_REGION_VIOLATION";}
        if(failures || k==last_k) break; // Preserve the failing row; do not hide it with continued zero commands.
        std::vector<double> next;
        if(!stepExplicitState(state,control,plant,liquid,plant.dt,next)) return 3;
        const double actual_jerk = std::abs(actualAcceleration(next[3],next[8],plant)
            -actualAcceleration(state[3],state[8],plant))/plant.dt;
        max_actual_jerk=std::max(max_actual_jerk,actual_jerk);
        if (params.actual_jerk_max>0 && actual_jerk>params.actual_jerk_max+1e-6/plant.dt) {
            ++failures; failure_status="PLANT_ACTUAL_JERK_VIOLATION"; break;
        }
        state=std::move(next);
    }
    boost::property_tree::ptree report;
    report.put("mode",mode);report.put("scenario_plan_id",plan.plan_id);
    report.put("deadline",plan.deadline);report.put("evaluation_window",plan.stop_window);
    report.put("curvature_weight",params.planning.geometry.curvature_weight);report.put("contour_weight",variant.w_contour);
    report.put("rti_iterations",params.rti_iterations);report.put("max_prediction_defect",max_defect);
    report.put("rti_min_iterations",params.rti_min_iterations);
    report.put("solve_budget_ms",options.solve_budget_ms);
    report.put("cycle_p95_ms",percentile(cycle_ms,.95));
    report.put("cycle_max_ms",cycle_ms.empty()?0.:*std::max_element(cycle_ms.begin(),cycle_ms.end()));
    report.put("actual_v_min",min_v); report.put("actual_v_max",max_v);
    report.put("actual_omega_min",min_omega); report.put("actual_omega_max",max_omega);
    report.put("walltime_p95_ms",percentile(wall_ms,.95)); report.put("walltime_max_ms",wall_ms.empty()?0.:*std::max_element(wall_ms.begin(),wall_ms.end()));
    report.put("solver_time_p95_ms",percentile(solver_ms,.95)); report.put("solver_time_max_ms",solver_ms.empty()?0.:*std::max_element(solver_ms.begin(),solver_ms.end()));
    report.put("max_actual_jerk",max_actual_jerk);report.put("actual_jerk_max",params.actual_jerk_max);
    report.put("max_jerk",max_jerk);report.put("max_solve_ms",max_solve_ms);report.put("evidence","MODEL_IN_THE_LOOP");report.put("actuator_scale",actuator_scale);
    const double lateness=completion_time<0 ? -1. : std::max(0.,completion_time-plan.deadline);
    const double deadline_tolerance=params.planning.trajectory.deadline_tolerance;
    const bool complete=completion_time>=0 && completion_time<=plan.deadline+deadline_tolerance+1e-9 && failures==0 && last_time>=end_time-1e-9;
    report.put("deadline_lateness_sec",lateness); report.put("deadline_tolerance_sec",deadline_tolerance);
    report.put("complete",complete);report.put("completion_time",completion_time);
    report.put("final_goal_position_error",std::hypot(state[0]-plan.goal_pose[0],state[1]-plan.goal_pose[1]));
    report.put("final_goal_yaw_error",std::abs(std::atan2(std::sin(state[2]-plan.goal_pose[2]),std::cos(state[2]-plan.goal_pose[2]))));
    report.put("evaluation_end_time",end_time); report.put("evaluation_last_sample_time",last_time);
    report.put("height_sampling","30 Hz model nodes; not continuous peak certification");
    report.add_child("effective_parameter_manifest",parameterManifest(params,variant,plan,first_snapshot));
    report.put("failures",failures);report.put("failure_status",failure_status);
    report.put("curvature_arc_energy",curvature_energy);report.put("path_length",distance);
    report.put("low_speed_spin_samples",spins);report.put("peak_modal_height_m",peak);
    report.put("minimum_region_clearance",minimum_clearance);
    boost::property_tree::write_json(prefix+".json",report);
    std::cout<<mode<<" completion="<<completion_time<<" failures="<<failures<<" reason="<<failure_status<<" curvature_energy="<<curvature_energy<<" distance="<<distance<<" peak="<<peak<<'\n';
    return complete?0:1;
}
}  // namespace

int main(int argc,char** argv) {
    try { return runTrial(parseOptions(argc,argv)); }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 2; }
}
