#include "spmpc_local_planner/analysis/g4_snapshot_replay.h"

#include "spmpc_local_planner/solver/acados/generated_solver.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace spmpc_local_planner {
namespace analysis {
namespace {

bool finiteValues(const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

bool validBounds(const SolverBoundSummary& bounds) {
    const double values[] = {
        bounds.a_min, bounds.a_max,
        bounds.alpha_min, bounds.alpha_max,
        bounds.v_s_min, bounds.v_s_max,
        bounds.v_min, bounds.v_max,
        bounds.omega_min, bounds.omega_max,
    };
    for (double value : values) {
        if (!std::isfinite(value)) return false;
    }
    return bounds.a_min <= bounds.a_max &&
           bounds.alpha_min <= bounds.alpha_max &&
           bounds.v_s_min <= bounds.v_s_max &&
           bounds.v_min <= bounds.v_max &&
           bounds.omega_min <= bounds.omega_max;
}

std::string validateFrame(const G4ReplayFrame& frame,
                          const GeneratedAcadosSolver& solver) {
    if (frame.pair_index < 0) return "NEGATIVE_PAIR_INDEX";
    if (frame.direction_code < 0 || frame.direction_code > 2) {
        return "INVALID_DIRECTION_CODE";
    }
    if (frame.horizon_steps != solver.horizonSteps() ||
        frame.state_width != solver.stateWidth() ||
        frame.control_width != solver.controlWidth() ||
        frame.parameter_width != solver.parameterWidth()) {
        return "GENERATED_DIMENSION_MISMATCH";
    }
    if (!std::isfinite(frame.dt) || frame.dt <= 0.0) return "INVALID_DT";
    for (double value : frame.initial_state) {
        if (!std::isfinite(value)) return "NONFINITE_INITIAL_STATE";
    }
    if (!validBounds(frame.runtime_bounds)) return "INVALID_RUNTIME_BOUNDS";
    const std::size_t state_count = static_cast<std::size_t>(
        (solver.horizonSteps() + 1) * solver.stateWidth());
    const std::size_t control_count = static_cast<std::size_t>(
        solver.horizonSteps() * solver.controlWidth());
    const std::size_t parameter_count = static_cast<std::size_t>(
        (solver.horizonSteps() + 1) * solver.parameterWidth());
    if (frame.stage_parameters.size() != parameter_count) {
        return "PARAMETER_COUNT_MISMATCH";
    }
    if (frame.initial_guess_states.size() != state_count) {
        return "STATE_GUESS_COUNT_MISMATCH";
    }
    if (frame.initial_guess_controls.size() != control_count) {
        return "CONTROL_GUESS_COUNT_MISMATCH";
    }
    if (!finiteValues(frame.stage_parameters) ||
        !finiteValues(frame.initial_guess_states) ||
        !finiteValues(frame.initial_guess_controls)) {
        return "NONFINITE_REPLAY_VECTOR";
    }
    if ((frame.direction_code == 0) != frame.modal_overrides.empty()) {
        return "CHECKPOINT_OVERRIDE_CONTRACT";
    }
    for (const auto& modal : frame.modal_overrides) {
        for (double value : modal) {
            if (!std::isfinite(value)) return "NONFINITE_MODAL_OVERRIDE";
        }
    }
    return std::string{};
}

bool applyBounds(GeneratedAcadosSolver& solver,
                 const SolverBoundSummary& bounds,
                 double* initial_state) {
    if (!solver.setStateBounds(0, initial_state, initial_state)) return false;
    double lower_control[3] = {
        bounds.a_min, bounds.alpha_min, bounds.v_s_min};
    double upper_control[3] = {
        bounds.a_max, bounds.alpha_max, bounds.v_s_max};
    for (int stage = 0; stage < solver.horizonSteps(); ++stage) {
        if (!solver.setControlBounds(
                stage, lower_control, upper_control)) return false;
    }
    double lower_state[2] = {bounds.v_min, bounds.omega_min};
    double upper_state[2] = {bounds.v_max, bounds.omega_max};
    for (int stage = 1; stage <= solver.horizonSteps(); ++stage) {
        if (!solver.setStateBounds(stage, lower_state, upper_state)) {
            return false;
        }
    }
    return true;
}

G4ReplaySolution solveFrame(
    GeneratedAcadosSolver& solver,
    const G4ReplayFrame& frame,
    const std::array<double, 4>* modal_override) {
    G4ReplaySolution solution;
    for (int stage = 0; stage <= solver.horizonSteps(); ++stage) {
        const double* parameters = frame.stage_parameters.data() +
            static_cast<std::size_t>(stage * solver.parameterWidth());
        const double* state = frame.initial_guess_states.data() +
            static_cast<std::size_t>(stage * solver.stateWidth());
        if (!solver.updateParameters(stage, parameters) ||
            !solver.setState(stage, state)) {
            solution.status = -2;
            return solution;
        }
        if (stage < solver.horizonSteps()) {
            const double* control = frame.initial_guess_controls.data() +
                static_cast<std::size_t>(stage * solver.controlWidth());
            if (!solver.setControl(stage, control)) {
                solution.status = -2;
                return solution;
            }
        }
    }
    std::array<double, 10> initial_state = frame.initial_state;
    if (modal_override != nullptr) {
        std::copy(modal_override->begin(), modal_override->end(),
                  initial_state.begin() + 6);
    }
    if (!applyBounds(solver, frame.runtime_bounds, initial_state.data())) {
        solution.status = -2;
        return solution;
    }
    solution.status = solver.solve();
    solution.states.reserve(static_cast<std::size_t>(
        (solver.horizonSteps() + 1) * solver.stateWidth()));
    solution.controls.reserve(static_cast<std::size_t>(
        solver.horizonSteps() * solver.controlWidth()));
    std::vector<double> state(
        static_cast<std::size_t>(solver.stateWidth()), 0.0);
    std::vector<double> control(
        static_cast<std::size_t>(solver.controlWidth()), 0.0);
    for (int stage = 0; stage <= solver.horizonSteps(); ++stage) {
        if (!solver.getState(stage, state.data())) {
            solution.status = -2;
            return solution;
        }
        solution.states.insert(
            solution.states.end(), state.begin(), state.end());
        if (stage < solver.horizonSteps()) {
            if (!solver.getControl(stage, control.data())) {
                solution.status = -2;
                return solution;
            }
            solution.controls.insert(
                solution.controls.end(), control.begin(), control.end());
        }
    }
    return solution;
}

}  // namespace

bool G4SnapshotReplayRunner::available() {
    GeneratedAcadosSolver solver;
    return solver.create(GeneratedAcadosSolver::Kind::SLOSH);
}

G4SequenceReplayResult G4SnapshotReplayRunner::run(
    const std::vector<G4ReplayFrame>& frames) {
    G4SequenceReplayResult result;
    if (frames.empty()) {
        result.detail = "EMPTY_REPLAY_SEQUENCE";
        return result;
    }
    GeneratedAcadosSolver base;
    GeneratedAcadosSolver branch;
    if (!base.create(GeneratedAcadosSolver::Kind::SLOSH) ||
        !branch.create(GeneratedAcadosSolver::Kind::SLOSH)) {
        result.detail = "ACADOS_SLOSH_CAPSULE_CREATE_FAILED";
        return result;
    }
    int previous_pair_index = -1;
    for (const G4ReplayFrame& frame : frames) {
        result.failed_pair_index = frame.pair_index;
        if (frame.pair_index <= previous_pair_index) {
            result.detail = "PAIR_INDEX_NOT_STRICTLY_INCREASING";
            return result;
        }
        previous_pair_index = frame.pair_index;
        const std::string validation = validateFrame(frame, base);
        if (!validation.empty()) {
            result.detail = validation;
            return result;
        }
        if (frame.direction_code != 0) {
            G4CheckpointReplay checkpoint;
            checkpoint.pair_index = frame.pair_index;
            checkpoint.direction_code = frame.direction_code;
            if (!branch.copyIterateFrom(base)) {
                result.detail = "ACTUAL_ITERATE_COPY_FAILED";
                return result;
            }
            checkpoint.actual = solveFrame(branch, frame, nullptr);
            checkpoint.counterfactuals.reserve(frame.modal_overrides.size());
            for (const auto& modal : frame.modal_overrides) {
                if (!branch.copyIterateFrom(base)) {
                    result.detail = "COUNTERFACTUAL_ITERATE_COPY_FAILED";
                    return result;
                }
                checkpoint.counterfactuals.push_back(
                    solveFrame(branch, frame, &modal));
            }
            result.checkpoints.push_back(std::move(checkpoint));
        }
        const G4ReplaySolution sequential = solveFrame(base, frame, nullptr);
        if (sequential.status != 0) {
            result.detail = "SEQUENTIAL_ACTUAL_SOLVE_FAILED_" +
                std::to_string(sequential.status);
            return result;
        }
    }
    result.success = true;
    result.detail = "OK";
    result.failed_pair_index = -1;
    return result;
}

}  // namespace analysis
}  // namespace spmpc_local_planner
