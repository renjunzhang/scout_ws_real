#include "spmpc_local_planner/controller/phase_solve_adapter.h"

#include <cstddef>

namespace spmpc_local_planner {

PhaseSolveView makePhaseSolveView(const SolverOutput& output,
                                  int terminal_step) {
    PhaseSolveView view;
    view.cmd_v = output.cmd_v;
    view.cmd_omega = output.cmd_omega;
    if (!output.predicted_horizon.valid || terminal_step < 0 ||
        static_cast<std::size_t>(terminal_step) >=
            output.predicted_horizon.states.size()) {
        return view;
    }

    const HorizonStateDebug& state = output.predicted_horizon.states[
        static_cast<std::size_t>(terminal_step)];
    view.terminal_state_available = true;
    view.terminal_robot.x = state.x;
    view.terminal_robot.y = state.y;
    view.terminal_robot.yaw = state.yaw;
    view.terminal_robot.v = state.v;
    view.terminal_robot.omega = state.omega;
    view.terminal_slosh.eta_x = state.eta_x;
    view.terminal_slosh.eta_x_dot = state.eta_x_dot;
    view.terminal_slosh.eta_y = state.eta_y;
    view.terminal_slosh.eta_y_dot = state.eta_y_dot;
    return view;
}

}  // namespace spmpc_local_planner
