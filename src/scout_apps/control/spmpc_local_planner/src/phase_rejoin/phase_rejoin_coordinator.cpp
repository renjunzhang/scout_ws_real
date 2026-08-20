#include "spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

bool finiteNonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool normalizedNear(double actual, double expected, double tolerance) {
    return std::isfinite(actual) && std::isfinite(expected) &&
        std::abs(actual - expected) <=
            tolerance * std::max(1.0, std::abs(expected));
}

}  // namespace

bool PhaseRejoinCoordinator::configure(const PhaseRejoinParams& params,
                                       std::string& error) {
    error.clear();
    const bool valid = params.liquid_horizon_steps > 0 &&
        finiteNonnegative(params.max_residual_v) &&
        finiteNonnegative(params.max_residual_omega) &&
        finiteNonnegative(params.artifact_dt_tolerance_sec) &&
        finiteNonnegative(params.artifact_path_length_tolerance_m) &&
        finiteNonnegative(params.artifact_path_geometry_tolerance_m) &&
        finiteNonnegative(params.artifact_model_tolerance) &&
        finiteNonnegative(params.artifact_command_tolerance) &&
        params.candidate.max_clock_lead_steps >= 0 &&
        selector_.configure(params.candidate);
    if (!valid) {
        configured_ = false;
        contract_valid_ = false;
        error = "INVALID_PHASE_REJOIN_PARAMS";
        return false;
    }
    params_ = params;
    configured_ = true;
    contract_valid_ = params.mode == PhaseRejoinMode::Off;
    contract_status_ = contract_valid_ ? "OFF" : "NOT_VALIDATED";
    resetProgress();
    return true;
}

NominalArtifactLoadResult PhaseRejoinCoordinator::loadArtifact(
    const std::string& path) {
    contract_valid_ = false;
    resetProgress();
    const NominalArtifactLoadResult result = artifact_.loadCsv(path);
    contract_status_ = result.success ? "ARTIFACT_LOADED" : result.status;
    return result;
}

bool PhaseRejoinCoordinator::setArtifact(
    const NominalSequenceArtifact& artifact,
    std::string& error) {
    error.clear();
    contract_valid_ = false;
    resetProgress();
    if (!artifact.valid() || artifact.empty()) {
        error = "ARTIFACT_INVALID";
        contract_status_ = error;
        return false;
    }
    artifact_ = artifact;
    contract_status_ = "ARTIFACT_LOADED";
    return true;
}

bool PhaseRejoinCoordinator::validateRuntimeContract(
    const PhaseRejoinRuntimeContract& runtime,
    const ReferencePath& reference,
    std::string& error) {
    error.clear();
    contract_valid_ = false;
    if (!configured_) {
        error = "NOT_CONFIGURED";
    } else if (params_.mode == PhaseRejoinMode::Off) {
        contract_valid_ = true;
        contract_status_ = "OFF";
        return true;
    } else if (!artifact_.valid()) {
        error = "ARTIFACT_UNAVAILABLE";
    } else if (!std::isfinite(runtime.dt) || runtime.dt <= 0.0 ||
               std::abs(runtime.dt - artifact_.metadata().dt) >
                   params_.artifact_dt_tolerance_sec) {
        error = "DT_MISMATCH";
    } else if (reference.empty()) {
        error = "PATH_UNAVAILABLE";
    } else if (!std::isfinite(reference.length()) ||
               reference.length() <= 0.0 ||
               std::abs(reference.length() -
                        artifact_.metadata().path_length) >
                   params_.artifact_path_length_tolerance_m) {
        error = "PATH_LENGTH_MISMATCH";
    } else if (!params_.required_contract_id.empty() &&
               params_.required_contract_id !=
                   artifact_.metadata().contract_id) {
        error = "CONTRACT_ID_MISMATCH";
    } else {
        if (reference.frameId().empty() ||
            reference.frameId() != artifact_.metadata().frame_id) {
            error = "FRAME_ID_MISMATCH";
        } else if (!params_.required_frame_id.empty() &&
                   (reference.frameId() != params_.required_frame_id ||
                    artifact_.metadata().frame_id !=
                        params_.required_frame_id)) {
            error = "REQUIRED_FRAME_ID_MISMATCH";
        } else if (params_.mode == PhaseRejoinMode::Enforce &&
                   artifact_.metadata().evidence_level ==
                       PhaseRejoinEvidenceLevel::DevelopmentOnly &&
                   !params_.allow_development_artifact_in_enforce) {
            error = "DEVELOPMENT_ARTIFACT_FORBIDDEN";
        } else if (params_.mode == PhaseRejoinMode::Enforce &&
                   (!artifact_.metadata().complete_terminal_tail ||
                    artifact_.metadata().terminal_zero_hold_steps <
                        static_cast<std::size_t>(
                            params_.liquid_horizon_steps + 2))) {
            error = "COMPLETE_TERMINAL_TAIL_REQUIRED";
        }
    }
    if (error.empty()) {
        for (const auto& sample : artifact_.samples()) {
            const TrajectoryPoint point = reference.sample(sample.s);
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                std::hypot(sample.x - point.x, sample.y - point.y) >
                    params_.artifact_path_geometry_tolerance_m) {
                error = "PATH_GEOMETRY_MISMATCH";
                break;
            }
        }
    }
    if (error.empty() &&
        artifact_.metadata().schema == "phase_rejoin_empirical_v2") {
        const auto& metadata = artifact_.metadata();
        if (!runtime.liquid_model_configured) {
            error = "RUNTIME_LIQUID_MODEL_UNAVAILABLE";
        } else if (!normalizedNear(metadata.two_zeta_omega_n,
                                   runtime.two_zeta_omega_n,
                                   params_.artifact_model_tolerance) ||
                   !normalizedNear(metadata.omega_n_sq,
                                   runtime.omega_n_sq,
                                   params_.artifact_model_tolerance) ||
                   !normalizedNear(metadata.kappa_x,
                                   runtime.kappa_x,
                                   params_.artifact_model_tolerance) ||
                   !normalizedNear(metadata.kappa_y,
                                   runtime.kappa_y,
                                   params_.artifact_model_tolerance)) {
            error = "LIQUID_MODEL_MISMATCH";
        }
    }
    if (error.empty()) {
        const double tolerance = params_.artifact_command_tolerance;
        const bool valid_bounds =
            std::isfinite(runtime.min_command_v) &&
            std::isfinite(runtime.max_command_v) &&
            std::isfinite(runtime.max_abs_command_omega) &&
            runtime.min_command_v <= runtime.max_command_v &&
            runtime.max_abs_command_omega >= 0.0;
        if (!valid_bounds) {
            error = "RUNTIME_COMMAND_BOUNDS_INVALID";
        } else {
            for (const auto& sample : artifact_.samples()) {
                const bool within =
                    sample.u_pub_v >= runtime.min_command_v - tolerance &&
                    sample.u_pub_v <= runtime.max_command_v + tolerance &&
                    std::abs(sample.u_pub_omega) <=
                        runtime.max_abs_command_omega + tolerance &&
                    sample.kappa_v >= runtime.min_command_v - tolerance &&
                    sample.kappa_v <= runtime.max_command_v + tolerance &&
                    std::abs(sample.kappa_omega) <=
                        runtime.max_abs_command_omega + tolerance;
                if (!within) {
                    error = "ARTIFACT_COMMAND_BOUNDS_MISMATCH";
                    break;
                }
            }
        }
    }
    if (!error.empty()) {
        contract_status_ = error;
        return false;
    }
    contract_valid_ = true;
    contract_status_ = "OK";
    resetProgress();
    return true;
}

void PhaseRejoinCoordinator::resetProgress() {
    have_accepted_index_ = false;
    accepted_index_ = 0;
    terminal_release_authorized_ = false;
    phase_clock_.reset();
}

PhaseNominalStage PhaseRejoinCoordinator::makeStage(
    const PhaseNominalSample& sample,
    bool gate_active) {
    PhaseNominalStage stage;
    stage.valid = true;
    stage.gate_active = gate_active;
    stage.artifact_index = sample.index;
    stage.x = sample.x;
    stage.y = sample.y;
    stage.yaw = sample.yaw;
    stage.s = sample.s;
    stage.v = sample.v;
    stage.omega = sample.omega;
    stage.eta_x = sample.eta_x;
    stage.eta_x_dot = sample.eta_x_dot;
    stage.eta_y = sample.eta_y;
    stage.eta_y_dot = sample.eta_y_dot;
    stage.a = sample.a;
    stage.alpha = sample.alpha;
    stage.v_s = sample.v_s;
    stage.radii = sample.radii;
    return stage;
}

PhaseRejoinPreparation PhaseRejoinCoordinator::prepare(
    const RobotState& execution_front_robot,
    const SloshState& execution_front_slosh,
    int front_steps,
    int solver_horizon_steps,
    double phase_time_sec,
    bool solver_origin_at_execution_front) {
    PhaseRejoinPreparation preparation;
    if (!configured_) {
        preparation.status = "NOT_CONFIGURED";
        return preparation;
    }
    if (params_.mode == PhaseRejoinMode::Off) {
        preparation.status = "OFF";
        return preparation;
    }
    if (!contract_valid_) {
        preparation.status = contract_status_;
        return preparation;
    }
    const int solver_terminal_step = params_.liquid_horizon_steps +
        (solver_origin_at_execution_front ? 0 : front_steps);
    if (front_steps < 0 || solver_terminal_step < 0 ||
        solver_horizon_steps < solver_terminal_step) {
        preparation.status = "SOLVER_HORIZON_TOO_SHORT";
        return preparation;
    }
    preparation.solver_terminal_step = solver_terminal_step;
    preparation.solver_origin_at_execution_front =
        solver_origin_at_execution_front;

    const std::size_t required_tail = static_cast<std::size_t>(
        front_steps + params_.liquid_horizon_steps);
    if (required_tail >= artifact_.size()) {
        preparation.status = "ARTIFACT_TOO_SHORT";
        return preparation;
    }
    const std::size_t max_current = artifact_.size() - required_tail - 1;
    const PhaseClockResult clock = phase_clock_.update(
        artifact_, phase_time_sec, max_current);
    if (!clock.valid) {
        preparation.status = clock.status;
        return preparation;
    }
    preparation.phase_clock_elapsed_sec = clock.elapsed_sec;

    preparation.candidate = selector_.select(
        artifact_, execution_front_robot, execution_front_slosh,
        front_steps, params_.liquid_horizon_steps,
        clock.index,
        have_accepted_index_, accepted_index_);
    if (!preparation.candidate.valid) {
        preparation.status = preparation.candidate.status;
        return preparation;
    }

    const PhaseNominalSample* current = artifact_.sample(
        preparation.candidate.current_index);
    const PhaseNominalSample* terminal = artifact_.sample(
        preparation.candidate.terminal_index);
    if (current == nullptr || terminal == nullptr) {
        preparation.status = "ARTIFACT_INDEX_MISSING";
        return preparation;
    }

    // Monitor is a strict shadow: it may inspect the same solve but must not
    // activate a solver-side phase objective or constraint.
    preparation.solver_context.active = true;
    preparation.solver_context.enforce =
        params_.mode == PhaseRejoinMode::Enforce;
    preparation.solver_context.empirical_gate = true;
    preparation.solver_context.state_complete_for_certificate = false;
    preparation.solver_context.owns_terminal_maneuver =
        params_.mode == PhaseRejoinMode::Enforce &&
        artifact_.metadata().complete_terminal_tail;
    preparation.solver_context.terminal_release_authorized =
        terminal_release_authorized_;
    preparation.solver_context.current_index =
        preparation.candidate.current_index;
    preparation.solver_context.front_index =
        preparation.candidate.front_index;
    preparation.solver_context.terminal_index =
        preparation.candidate.terminal_index;
    preparation.solver_context.front_steps = front_steps;
    preparation.solver_context.liquid_steps =
        params_.liquid_horizon_steps;
    preparation.solver_context.stages.reserve(
        static_cast<std::size_t>(params_.liquid_horizon_steps + 1));
    for (int k = 0; k <= params_.liquid_horizon_steps; ++k) {
        const std::size_t index = preparation.candidate.front_index +
            static_cast<std::size_t>(k);
        const PhaseNominalSample* sample = artifact_.sample(index);
        if (sample == nullptr) {
            preparation.status = "NOMINAL_HORIZON_INCOMPLETE";
            preparation.solver_context = PhaseRejoinSolverContext{};
            return preparation;
        }
        preparation.solver_context.stages.push_back(makeStage(
            *sample, k == params_.liquid_horizon_steps));
    }
    const PhaseNominalSample* front = artifact_.sample(
        preparation.candidate.front_index);
    if (front == nullptr) {
        preparation.status = "EXECUTION_FRONT_COMMAND_MISSING";
        preparation.solver_context = PhaseRejoinSolverContext{};
        return preparation;
    }
    preparation.nominal_cmd_v = front->u_pub_v;
    preparation.nominal_cmd_omega = front->u_pub_omega;
    preparation.solver_context.nominal_publish_v = front->u_pub_v;
    preparation.solver_context.nominal_publish_omega = front->u_pub_omega;
    preparation.solver_context.max_residual_v = params_.max_residual_v;
    preparation.solver_context.max_residual_omega = params_.max_residual_omega;
    preparation.recovery_cmd_v = front->kappa_v;
    preparation.recovery_cmd_omega = front->kappa_omega;
    preparation.ready = true;
    preparation.command_intervention_allowed =
        params_.mode == PhaseRejoinMode::Enforce;
    preparation.status = params_.mode == PhaseRejoinMode::Monitor
        ? "MONITOR_READY"
        : "ENFORCE_READY";
    return preparation;
}

RobotState PhaseRejoinCoordinator::robotFromHorizon(
    const HorizonStateDebug& state) {
    RobotState robot;
    robot.x = state.x;
    robot.y = state.y;
    robot.yaw = state.yaw;
    robot.v = state.v;
    robot.omega = state.omega;
    return robot;
}

SloshState PhaseRejoinCoordinator::sloshFromHorizon(
    const HorizonStateDebug& state) {
    SloshState slosh;
    slosh.eta_x = state.eta_x;
    slosh.eta_x_dot = state.eta_x_dot;
    slosh.eta_y = state.eta_y;
    slosh.eta_y_dot = state.eta_y_dot;
    return slosh;
}

PhaseRejoinDecision PhaseRejoinCoordinator::decide(
    const PhaseRejoinPreparation& preparation,
    const RobotState& execution_front_robot,
    const SloshState& execution_front_slosh,
    bool solver_success,
    const SolverOutput& solver_output) const {
    PhaseRejoinDecision decision;
    decision.solver_cmd_v = solver_output.cmd_v;
    decision.solver_cmd_omega = solver_output.cmd_omega;
    decision.output_cmd_v = solver_output.cmd_v;
    decision.output_cmd_omega = solver_output.cmd_omega;
    if (!preparation.ready || !preparation.candidate.valid) {
        decision.status = preparation.status;
        if (params_.mode == PhaseRejoinMode::Enforce) {
            decision.output_cmd_v = 0.0;
            decision.output_cmd_omega = 0.0;
            decision.command_intervened = true;
            decision.controlled_stop_used = true;
            decision.status = "ENFORCE_NOT_READY_STOP_" + preparation.status;
        }
        return decision;
    }

    const PhaseNominalSample* front = artifact_.sample(
        preparation.candidate.front_index);
    const PhaseNominalSample* terminal = artifact_.sample(
        preparation.candidate.terminal_index);
    if (front == nullptr || terminal == nullptr) {
        decision.status = "ARTIFACT_INDEX_MISSING";
        return decision;
    }
    decision.current_gate = gate_.evaluate(
        *front, execution_front_robot, execution_front_slosh);
    decision.current_gate_accepted = decision.current_gate.accepted;

    const int terminal_step = preparation.solver_terminal_step;
    if (solver_output.predicted_horizon.valid && terminal_step >= 0 &&
        static_cast<std::size_t>(terminal_step) <
            solver_output.predicted_horizon.states.size()) {
        const HorizonStateDebug& terminal_state =
            solver_output.predicted_horizon.states[
                static_cast<std::size_t>(terminal_step)];
        decision.terminal_gate = gate_.evaluate(
            *terminal, robotFromHorizon(terminal_state),
            sloshFromHorizon(terminal_state));
    } else {
        decision.terminal_gate.status = "HORIZON_UNAVAILABLE";
    }
    decision.terminal_gate_accepted = decision.terminal_gate.accepted;
    decision.evaluated = true;

    if (params_.mode == PhaseRejoinMode::Monitor) {
        decision.status = decision.terminal_gate_accepted
            ? "MONITOR_TERMINAL_ACCEPTED"
            : "MONITOR_TERMINAL_REJECTED";
        return decision;
    }
    if (params_.mode != PhaseRejoinMode::Enforce) {
        decision.status = "OFF";
        return decision;
    }

    if (solver_success && decision.terminal_gate_accepted) {
        decision.residual_v = solver_output.cmd_v - preparation.nominal_cmd_v;
        decision.residual_omega =
            solver_output.cmd_omega - preparation.nominal_cmd_omega;
        const bool residual_consistent =
            std::abs(decision.residual_v) <= params_.max_residual_v + 1e-7 &&
            std::abs(decision.residual_omega) <=
                params_.max_residual_omega + 1e-7;
        if (!residual_consistent) {
            decision.output_cmd_v = 0.0;
            decision.output_cmd_omega = 0.0;
            decision.command_intervened = true;
            decision.controlled_stop_used = true;
            decision.status = "ENFORCE_SOLVER_COMMAND_CONTRACT_VIOLATION";
            return decision;
        }
        // The successful solver command is already residual-constrained in the
        // OCP.  Never mutate a command after validating its predicted horizon.
        decision.command_contract_consistent = true;
        decision.output_cmd_v = solver_output.cmd_v;
        decision.output_cmd_omega = solver_output.cmd_omega;
        decision.command_intervened = false;
        decision.status = "ENFORCE_TERMINAL_ACCEPTED";
        return decision;
    }

    if (decision.current_gate_accepted) {
        decision.output_cmd_v = preparation.recovery_cmd_v;
        decision.output_cmd_omega = preparation.recovery_cmd_omega;
        decision.command_intervened = true;
        decision.recovery_command_used = true;
        decision.status = solver_success
            ? "ENFORCE_TERMINAL_REJECTED_RECOVERY"
            : "ENFORCE_SOLVER_FAILED_RECOVERY";
        return decision;
    }

    decision.output_cmd_v = 0.0;
    decision.output_cmd_omega = 0.0;
    decision.command_intervened = true;
    decision.controlled_stop_used = true;
    decision.status = solver_success
        ? "ENFORCE_TERMINAL_REJECTED_STOP"
        : "ENFORCE_SOLVER_FAILED_STOP";
    return decision;
}

void PhaseRejoinCoordinator::commit(
    const PhaseRejoinPreparation& preparation,
    const PhaseRejoinDecision& decision) {
    if (!preparation.ready || !preparation.candidate.valid) {
        return;
    }
    if (params_.mode == PhaseRejoinMode::Monitor ||
        (params_.mode == PhaseRejoinMode::Enforce &&
         decision.terminal_gate_accepted &&
         decision.command_contract_consistent)) {
        accepted_index_ = preparation.candidate.current_index;
        have_accepted_index_ = true;
    }
    if (params_.mode == PhaseRejoinMode::Enforce &&
        preparation.solver_context.owns_terminal_maneuver &&
        decision.terminal_gate_accepted &&
        decision.command_contract_consistent &&
        preparation.candidate.terminal_index + 1 == artifact_.size()) {
        terminal_release_authorized_ = true;
    }
}

PhaseRejoinDebugData PhaseRejoinCoordinator::makeDebug(
    const PhaseRejoinPreparation* preparation,
    const PhaseRejoinDecision* decision) const {
    PhaseRejoinDebugData debug;
    debug.mode = params_.mode;
    debug.artifact_loaded = artifact_.valid();
    debug.contract_valid = contract_valid_;
    debug.artifact_size = artifact_.size();
    debug.empirical_gate = artifact_.valid();
    debug.state_complete_for_certificate = false;
    debug.status = contract_status_;
    debug.terminal_release_authorized = terminal_release_authorized_;
    if (artifact_.valid()) {
        debug.evidence_level = artifact_.metadata().evidence_level;
        debug.contract_id = artifact_.metadata().contract_id;
        debug.artifact_path = artifact_.path();
    }
    if (preparation != nullptr) {
        debug.ready = preparation->ready;
        debug.status = preparation->status;
        debug.nominal_cmd_v = preparation->nominal_cmd_v;
        debug.nominal_cmd_omega = preparation->nominal_cmd_omega;
        if (preparation->candidate.valid) {
            debug.current_index = preparation->candidate.current_index;
            debug.clock_index = preparation->candidate.clock_index;
            debug.front_index = preparation->candidate.front_index;
            debug.terminal_index = preparation->candidate.terminal_index;
            debug.candidate_count = preparation->candidate.candidate_count;
            debug.phase_lead_steps = preparation->candidate.phase_lead_steps;
            debug.candidate_score = preparation->candidate.score;
            debug.front_steps = preparation->solver_context.front_steps;
            debug.liquid_steps = preparation->solver_context.liquid_steps;
            debug.solver_terminal_step = preparation->solver_terminal_step;
            debug.solver_origin_at_execution_front =
                preparation->solver_origin_at_execution_front;
        }
    }
    if (decision != nullptr) {
        debug.status = decision->status;
        debug.terminal_gate_metric = decision->terminal_gate.metric;
        debug.current_gate_metric = decision->current_gate.metric;
        debug.terminal_gate_accepted = decision->terminal_gate_accepted;
        debug.current_gate_accepted = decision->current_gate_accepted;
        debug.command_intervened = decision->command_intervened;
        debug.recovery_command_used = decision->recovery_command_used;
        debug.controlled_stop_used = decision->controlled_stop_used;
        debug.command_contract_consistent =
            decision->command_contract_consistent;
        debug.solver_cmd_v = decision->solver_cmd_v;
        debug.solver_cmd_omega = decision->solver_cmd_omega;
        debug.output_cmd_v = decision->output_cmd_v;
        debug.output_cmd_omega = decision->output_cmd_omega;
        debug.residual_v = decision->residual_v;
        debug.residual_omega = decision->residual_omega;
    }
    return debug;
}

}  // namespace spmpc_local_planner
