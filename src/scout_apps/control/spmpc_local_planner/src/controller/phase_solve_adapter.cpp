#include "spmpc_local_planner/controller/phase_solve_adapter.h"

#include <cstddef>

namespace spmpc_local_planner {

PhaseSolveView makePhaseSolveView(const SolverOutput& output,
                                  int terminal_step,
                                  const ExecutionAugmentedState*
                                      known_initial_execution) {
    PhaseSolveView view;
    view.cmd_v = output.cmd_v;
    view.cmd_omega = output.cmd_omega;
    view.optimization_failure_recovery_eligible =
        output.failure_kind == SolverFailureKind::Optimization;
    if (known_initial_execution != nullptr &&
        known_initial_execution->valid) {
        view.current_execution_state_available = true;
        view.current_execution = *known_initial_execution;
    }
    if (output.delay_augmented_execution_solution) {
        // The context initial state is aligned from the published-command
        // history and is the controller's trusted input.  A decoded stage-0
        // solver value may be used only when that input is unavailable; it
        // must never replace the state used by the current compatibility gate.
        if (!view.current_execution_state_available &&
            output.initial_execution_state.valid) {
            view.current_execution_state_available = true;
            view.current_execution = output.initial_execution_state;
        }
        view.successor_execution_state_available =
            output.successor_execution_state.valid;
        view.successor_execution = output.successor_execution_state;
        view.terminal_execution_state_available =
            output.terminal_execution_state.valid;
        view.terminal_execution = output.terminal_execution_state;
    }
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
