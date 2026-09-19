// Reconstruct feedback from an archived native command trace, then cold solve once.
// This cannot restore missing dual/QP memory or prove the cause of an old failure.
#include "geometry_trial_support.h"
#include "spmpc_local_planner/dynamics/explicit_state_rollout.h"
#include <fstream>
#include <iostream>
#include <sstream>

using namespace spmpc_local_planner;
using namespace spmpc_native;
using boost::property_tree::ptree;

namespace {
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> values;
    std::istringstream stream(line); std::string value;
    while (std::getline(stream,value,',')) values.push_back(value);
    return values;
}
ptree array(const std::vector<double>& values) {
    ptree result;
    for (double value:values) { ptree item; item.put_value(value); result.push_back({"",item}); }
    return result;
}
int run(int argc,char** argv) {
    if (argc!=5) throw std::invalid_argument("usage: replay_geometry_state plan.json native.csv time output.json");
    const double target=std::stod(argv[3]);
    if (!std::isfinite(target) || target<0.) throw std::invalid_argument("invalid replay time");
    const auto plan=TrajectoryPlan::load(argv[1]);
    Options options; options.mode="planned_slosh"; options.plan_file=argv[1];
    options.contour_weight=1.; options.goal_weight=0.; options.progress_weight=0.;
    // Isolate the reset candidate from quality-based early exit and live timing.
    options.rti_iterations=3; options.rti_min_iterations=3;
    const auto config=makeConfig(options,plan);
    SloshDynamics liquid;
    if (!liquid.configure(config.params.slosh)) throw std::runtime_error("liquid model invalid");
    std::vector<double> state=taskInitialState(argv[1],plan);
    std::ifstream file(argv[2]); std::string line;
    if (!std::getline(file,line) || line!="t,x,y,yaw,v,omega,v_cmd,omega_cmd,a_cmd,alpha_cmd,vs,height,status,projected_s,ref_x,ref_y,ref_yaw")
        throw std::invalid_argument("unsupported native trace columns");
    const size_t indices[]={0,1,2,3,5,6,7};
    double max_state_error=0.,max_height_error=0.,progress=0.;
    size_t k=0; bool found=false; std::string original_status;
    while (std::getline(file,line)) {
        const auto row=split(line);
        if (row.size()!=17) throw std::invalid_argument("incomplete native trace row");
        const double time=std::stod(row[0]);
        if (std::abs(time-k*plan.dt)>1e-8) throw std::invalid_argument("noncontiguous command history");
        for (size_t i=0;i<7;++i) {
            const double measured=std::stod(row[i+1]);
            if (!std::isfinite(measured)) throw std::invalid_argument("nonfinite recorded state");
            max_state_error=std::max(max_state_error,std::abs(state[indices[i]]-measured));
        }
        const auto input=makeInput(state,time);
        max_height_error=std::max(max_height_error,std::abs(liquid.height(input.slosh)-std::stod(row[11])));
        if (max_state_error>1e-9 || max_height_error>1e-9)
            throw std::runtime_error("command replay does not reproduce archived feedback");
        if (std::abs(time-target)<1e-8) {
            found=true; progress=std::stod(row[13]); original_status=row[12]; break;
        }
        if (time>target) break;
        std::array<double,3> control{{std::stod(row[8]),std::stod(row[9]),std::stod(row[10])}};
        std::vector<double> next;
        if (!stepExplicitState(state,control,config.params.actuator,liquid,plan.dt,next))
            throw std::runtime_error("command replay dynamics failure");
        state=std::move(next); ++k;
    }
    if (!found) throw std::invalid_argument("target time not in trace");
    std::vector<TrajectoryPoint> points;
    for (size_t i=0;i<plan.route.size();++i) {
        const auto& xy=plan.route[i];
        const double yaw=i+1<plan.route.size() ?
            std::atan2(plan.route[i+1].y-xy.y,plan.route[i+1].x-xy.x) : plan.goal_pose[2];
        points.push_back({xy.x,xy.y,yaw,0.,0.});
    }
    ReferencePath route; route.setPoints(points,plan.frame_id);
    // A fresh problem/capsule has no old QP memory. Production warm-start code
    // re-rolls the trajectory plan from this exact reconstructed feedback.
    SpmpcProblem problem; problem.configure(config.params,config.variant); problem.setReferencePath(route);
    if (!problem.configurationError().empty()) throw std::runtime_error(problem.configurationError());
    const auto input=makeInput(state,target);
    SolverOutput out;
    const auto begin=SolveBudget::Clock::now();
    const bool ok=problem.solve(input,out);
    const double wall_ms=std::chrono::duration<double,std::milli>(SolveBudget::Clock::now()-begin).count();
    const auto& snap=out.pre_solve_snapshot;
    std::vector<double> expected=state;
    if (snap.valid) expected[4]=snap.s0; // s is the projected virtual progress, not measured motion.
    double anchor_error=0.;
    const bool have_anchor=!snap.initial_guess_states.empty() &&
        snap.initial_guess_states.front().model_state.size()==expected.size();
    if (have_anchor) for (size_t i=0;i<expected.size();++i)
        anchor_error=std::max(anchor_error,std::abs(expected[i]-snap.initial_guess_states.front().model_state[i]));
    ptree report;
    report.put("evidence","RECONSTRUCTED_FEEDBACK_COLD_START_SCREEN_NOT_EXACT_SOLVER_REPLAY");
    report.put("original_status",original_status); report.put("time",target);
    report.put("replayed_control_steps",k); report.put("feedback_max_error",max_state_error);
    report.put("height_max_error",max_height_error); report.add_child("reconstructed_state",array(state));
    report.put("recorded_progress",progress); report.put("cold_solve_progress",snap.s0);
    report.put("fresh_guess_source",snap.warm_start_source);
    report.put("physical_state_and_fifo_anchored",have_anchor && anchor_error<1e-9);
    report.put("initial_guess_anchor_error",anchor_error);
    report.put("status",out.status); report.put("solver_status",snap.solver_status);
    report.put("rti_iterations",snap.rti_iterations); report.put("wall_ms",wall_ms);
    report.put("complete_validation_passed",ok && out.success && out.predicted_horizon.valid);
    report.put("prediction_defect",out.predicted_horizon.dynamics_max_defect);
    report.put("actual_jerk_max",out.predicted_horizon.actual_jerk_max);
    report.put("cost_reconstruction_valid",out.cost.reconstruction_valid);
    report.put("limitations","Old primal/dual/QP memory, terminal internal state and virtual-control cost history were not archived. Physical feedback/FIFO is reconstructed and checked; no root-cause or reset-at-recovery claim.");
    boost::property_tree::write_json(argv[4],report);
    std::cout<<out.status<<" cold_screen="<<(ok && out.success && out.predicted_horizon.valid)<<'\n';
    return ok && out.success && out.predicted_horizon.valid && have_anchor && anchor_error<1e-9 ? 0 : 1;
}
}
int main(int argc,char** argv) {
    try { return run(argc,argv); }
    catch (const std::exception& error) { std::cerr<<error.what()<<'\n'; return 2; }
}
