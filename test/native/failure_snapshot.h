#pragma once
#include "spmpc_local_planner/core/types.h"
#include <boost/property_tree/json_parser.hpp>

namespace spmpc_native {
template<class Values> boost::property_tree::ptree snapshotArray(const Values& values) {
    boost::property_tree::ptree result;
    for (const auto& value:values) {
        boost::property_tree::ptree item; item.put_value(value); result.push_back({"",item});
    }
    return result;
}
inline void saveFailureSnapshot(const std::string& file, double time,
    const std::vector<double>& state, const spmpc_local_planner::SolverOutput& out) {
    using boost::property_tree::ptree;
    const auto& s=out.pre_solve_snapshot;
    ptree data;
    data.put("schema","native_failure_v1"); data.put("task_elapsed_sec",time);
    data.put("status",out.status); data.put("solver_status",s.solver_status);
    data.put("limitation","Full feedback and primal/stage data; acados dual/QP memory not captured.");
    data.add_child("feedback_state",snapshotArray(state));
    data.put("dt",s.dt); data.put("rti_iterations",s.rti_iterations);
    data.put("qp_iteration_limit",s.qp_iteration_limit);
    data.put("min_progress_s",s.min_progress_s); data.put("s0",s.s0);
    data.put("state_width",s.state_width); data.put("control_width",s.control_width);
    data.put("parameter_width",s.parameter_width); data.put("horizon_steps",s.horizon_steps);
    data.put("actual_jerk_max",s.actual_jerk_max); data.put("jerk_max",s.jerk_max);
    data.put("max_prediction_defect",s.max_prediction_defect);
    data.put("warm_start_source",s.warm_start_source);
    data.put("have_previous_control",s.have_previous_control);
    data.add_child("previous_control",snapshotArray(std::array<double,3>{{s.previous_a,s.previous_alpha_or_omega,s.previous_v_s}}));
    data.add_child("parameter_names",snapshotArray(s.parameter_names));
    data.add_child("stage_parameters",snapshotArray(s.stage_parameters));
    data.add_child("terminal_command_caps",snapshotArray(s.terminal_command_caps));
    auto save_states=[&](const char* key,const std::vector<spmpc_local_planner::HorizonStateDebug>& rows) {
        ptree values; for (const auto& row:rows) values.push_back({"",snapshotArray(row.model_state)});
        data.add_child(key,values);
    };
    auto save_controls=[&](const char* key,const std::vector<spmpc_local_planner::HorizonControlDebug>& rows) {
        ptree values;
        for (const auto& row:rows) values.push_back({"",snapshotArray(std::array<double,3>{{row.a,row.alpha_or_omega,row.v_s}})});
        data.add_child(key,values);
    };
    save_states("initial_guess_states",s.initial_guess_states);
    save_controls("initial_guess_controls",s.initial_guess_controls);
    save_states("previous_solution_states",s.previous_solution_states);
    save_controls("previous_solution_controls",s.previous_solution_controls);
    boost::property_tree::write_json(file,data);
}
}  // namespace spmpc_native
