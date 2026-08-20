#include "spmpc_local_planner/controller/control_cycle_engine.h"
#include "spmpc_local_planner/controller/phase_solve_adapter.h"

#include <cmath>

namespace spmpc_local_planner {

ControlCycleEngine::ControlCycleEngine(SolverSession& solver_session)
    : solver_session_(solver_session) {}

bool ControlCycleEngine::configurePhaseRejoin(
    const PhaseRejoinParams& params,
    std::string& error) {
    if (!phase_rejoin_.configure(params, error)) {
        return false;
    }
    phase_params_ = params;
    goal_reached_latched_ = false;
    return true;
}

NominalArtifactLoadResult ControlCycleEngine::loadPhaseRejoinArtifact(
    const std::string& path) {
    return phase_rejoin_.loadArtifact(path);
}

bool ControlCycleEngine::validatePhaseRejoinRuntimeContract(
    const PhaseRejoinRuntimeContract& runtime,
    const ReferencePath& reference,
    std::string& error) {
    return phase_rejoin_.validateRuntimeContract(runtime, reference, error);
}

bool ControlCycleEngine::configureSafety(
    const SafetySupervisorConfig& config,
    std::string& error) {
    return safety_.configure(config, error);
}

bool ControlCycleEngine::configureCommandPipeline(
    const CommandPipelineConfig& config,
    std::string& error) {
    return command_pipeline_.configure(config, error);
}

SpeedReferenceConfigureResult ControlCycleEngine::configureSpeedReference(
    const SpeedReferenceControllerConfig& config) {
    return speed_reference_.configure(config);
}

SpeedReferenceEvaluation ControlCycleEngine::prepareSpeedReference(
    SolverInput& input) {
    return speed_reference_.apply(input.robot, input.slosh, input);
}

CommandPipelineResult ControlCycleEngine::finalizeCommand(
    const CommandDecision& decision,
    StampNs stamp_ns,
    bool publish_enabled) {
    CommandPipelineRequest request;
    request.stamp_ns = stamp_ns;
    request.desired = decision.command;
    request.publish_enabled = publish_enabled;
    request.source = decision.source;
    request.reason = decision.reason;
    return command_pipeline_.finalize(request);
}

CommandPipelineResult ControlCycleEngine::finalizeFailClosedZero(
    StampNs stamp_ns,
    bool publish_enabled,
    const std::string& reason) {
    CommandPipelineRequest request;
    request.stamp_ns = stamp_ns;
    request.force_zero = true;
    request.publish_enabled = publish_enabled;
    request.source = CommandSource::FailClosed;
    request.reason = reason.empty() ? "FAIL_CLOSED_ZERO" : reason;
    return command_pipeline_.finalize(request);
}

void ControlCycleEngine::resetSafety() {
    safety_.reset();
}

void ControlCycleEngine::resetForReference() {
    safety_.reset();
    phase_rejoin_.resetProgress();
    speed_reference_.resetForReference();
    goal_reached_latched_ = false;
    have_previous_shifted_plan_ = false;
}

bool ControlCycleEngine::phaseRejoinContractValid() const {
    return phase_rejoin_.contractValid();
}

PhaseRejoinDebugData ControlCycleEngine::makePhaseRejoinDebug(
    const PhaseRejoinPreparation* preparation,
    const PhaseRejoinDecision* decision) const {
    return phase_rejoin_.makeDebug(preparation, decision);
}

VelocityCommand ControlCycleEngine::rawSolverCommand(
    const SolverOutput& output) {
    VelocityCommand command;
    command.linear = output.first_shot_debug.success
        ? output.first_shot_debug.cmd_v_post_clamp
        : output.cmd_v;
    command.angular = output.first_shot_debug.success
        ? output.first_shot_debug.cmd_omega_post_clamp
        : output.cmd_omega;
    return command;
}

TrackingProjectionView ControlCycleEngine::projectionView(
    const ProjectorDebugSummary& projector) {
    TrackingProjectionView view;
    view.raw_valid = projector.raw_valid;
    view.raw_distance_m = projector.raw_distance;
    view.guarded_valid = projector.guarded_valid;
    view.guarded_distance_m = projector.guarded_distance;
    return view;
}

PhaseRejoinPreparation ControlCycleEngine::preparePhase(
    const ControlCycleRequest& request) {
    PhaseRejoinPreparation preparation;
    if (phase_params_.mode == PhaseRejoinMode::Off) {
        preparation.status = "OFF";
    } else if (!request.prediction_valid) {
        preparation.status = "PREDICTION_" + request.prediction_status;
    } else if (phase_params_.mode == PhaseRejoinMode::Enforce &&
               !request.solver_origin_at_execution_front) {
        preparation.status = "EXECUTION_FRONT_NOT_APPLIED";
    } else {
        preparation = phase_rejoin_.prepare(
            request.execution_front_robot,
            request.execution_front_slosh,
            request.execution_front_steps,
            request.solver_input.horizon_steps,
            request.phase_time_sec,
            request.solver_origin_at_execution_front);
    }
    return preparation;
}

ControlCycleResult ControlCycleEngine::step(
    const ControlCycleRequest& request) {
    ControlCycleResult result;
    result.solver_input = request.solver_input;
    result.phase_preparation = preparePhase(request);
    result.solver_input.phase_rejoin =
        result.phase_preparation.solver_context;

    result.solve_returned = solver_session_.solve(
        result.solver_input, result.solver_output);
    result.output = result.solver_output;
    result.solver_success = result.output.success;
    result.solver_command = rawSolverCommand(result.output);

    const bool reached_this_cycle =
        result.output.terminal_diagnostics.reached ||
        result.output.status == "GOAL_REACHED";
    if (reached_this_cycle) {
        goal_reached_latched_ = true;
    }
    if (goal_reached_latched_) {
        result.output.cmd_v = 0.0;
        result.output.cmd_omega = 0.0;
        result.output.success = true;
        result.output.status = reached_this_cycle
            ? "GOAL_REACHED"
            : "GOAL_REACHED_LATCHED";
    }
    result.terminal_command.linear = result.output.cmd_v;
    result.terminal_command.angular = result.output.cmd_omega;

    result.terminal_priority = goal_reached_latched_ ||
        result.output.terminal_diagnostics.reached ||
        result.output.status == "GOAL_REACHED";
    if (phase_params_.mode != PhaseRejoinMode::Off) {
        result.have_phase_decision = true;
        if (result.terminal_priority) {
            result.phase_decision.solver_cmd_v = result.output.cmd_v;
            result.phase_decision.solver_cmd_omega = result.output.cmd_omega;
            result.phase_decision.output_cmd_v = result.output.cmd_v;
            result.phase_decision.output_cmd_omega = result.output.cmd_omega;
            result.phase_decision.status = "BYPASSED_TERMINAL_PRIORITY";
        } else {
            result.phase_decision = phase_rejoin_.decide(
                result.phase_preparation,
                request.execution_front_robot,
                request.execution_front_slosh,
                result.solver_success,
                makePhaseSolveView(
                    result.solver_output,
                    result.phase_preparation.solver_terminal_step));
            if (phase_params_.mode == PhaseRejoinMode::Enforce) {
                result.output.cmd_v =
                    result.phase_decision.output_cmd_v;
                result.output.cmd_omega =
                    result.phase_decision.output_cmd_omega;
                result.output.status = result.phase_decision.status;
                result.output.success =
                    (result.solver_success &&
                     result.phase_decision.terminal_gate_accepted) ||
                    result.phase_decision.recovery_command_used;
                if (result.phase_decision.controlled_stop_used) {
                    result.output.success = false;
                    result.output.cmd_v = 0.0;
                    result.output.cmd_omega = 0.0;
                }
            }
        }
    }
    result.post_phase_command.linear = result.output.cmd_v;
    result.post_phase_command.angular = result.output.cmd_omega;

    SafetySupervisorInput safety_input;
    safety_input.robot = result.solver_input.robot;
    safety_input.command = result.post_phase_command;
    safety_input.command_accepted = result.output.success;
    safety_input.status = result.output.status;
    safety_input.terminal = result.output.terminal_diagnostics;
    safety_input.projection = projectionView(
        result.output.projector_debug);
    safety_input.period_sec = request.period_sec;
    result.safety = safety_.step(safety_input);
    if (result.safety.blocked) {
        result.output.success = false;
        result.output.status = result.safety.status;
        result.output.cmd_v = result.safety.command.linear;
        result.output.cmd_omega = result.safety.command.angular;
    }

    if (result.have_phase_decision && !result.terminal_priority &&
        !result.safety.blocked &&
        (phase_params_.mode == PhaseRejoinMode::Monitor ||
         result.output.success)) {
        phase_rejoin_.commit(
            result.phase_preparation, result.phase_decision);
    }

    result.terminal_controller_intervened =
        std::abs(result.terminal_command.linear -
                 result.solver_command.linear) > 1e-6 ||
        std::abs(result.terminal_command.angular -
                 result.solver_command.angular) > 1e-6;

    CommandArbitrationRequest arbitration;
    arbitration.safety.active = result.safety.blocked;
    arbitration.safety.accepted = false;
    arbitration.safety.command = VelocityCommand{};
    arbitration.safety.reason = result.safety.status;

    arbitration.terminal_reached.active = result.terminal_priority;
    arbitration.terminal_reached.accepted = true;
    arbitration.terminal_reached.command = result.terminal_command;
    arbitration.terminal_reached.reason = result.output.status;

    arbitration.phase_rejoin.active =
        phase_params_.mode == PhaseRejoinMode::Enforce &&
        result.have_phase_decision && !result.terminal_priority &&
        !result.safety.blocked && result.output.success;
    arbitration.phase_rejoin.accepted = result.output.success;
    arbitration.phase_rejoin.command = result.post_phase_command;
    arbitration.phase_rejoin.reason = result.output.status;

    arbitration.terminal.active = result.solver_success &&
        result.terminal_controller_intervened;
    arbitration.terminal.accepted = result.solver_success;
    arbitration.terminal.command = result.terminal_command;
    arbitration.terminal.reason = result.solver_output.status;

    arbitration.solver.active = result.solver_success;
    arbitration.solver.accepted = result.solver_success;
    // Preserve the exact command returned by the planning session even when
    // its difference from the raw first-shot command is below the audit
    // intervention tolerance.
    arbitration.solver.command = result.terminal_command;
    arbitration.solver.reason = result.solver_output.status;

    result.decision = arbitrateCommand(arbitration);
    result.output.cmd_v = result.decision.command.linear;
    result.output.cmd_omega = result.decision.command.angular;
    result.output.success = result.decision.accepted;

    speed_reference_.commitProgress(result.output.progress_abs_s);

    // Freeze the complete controller-owned decision trace before returning to
    // any transport adapter.  ROS may add observer/publish timing, but it no
    // longer needs to infer solver, terminal, phase or safety interventions.
    result.telemetry.cycle_id = request.cycle_id;
    result.telemetry.cycle_start_ns = request.cycle_start_ns;
    result.telemetry.status = result.output.status;
    result.telemetry.solver_status = result.solver_output.status;
    result.telemetry.command_reason = result.decision.reason;
    result.telemetry.command_source = result.decision.source;
    result.telemetry.solve_attempted = true;
    result.telemetry.solve_returned = result.solve_returned;
    result.telemetry.solve_success = result.solver_success;
    result.telemetry.command_accepted = result.output.success;
    result.telemetry.terminal_phase =
        result.output.terminal_diagnostics.terminal_phase;
    result.telemetry.terminal_priority = result.terminal_priority;
    result.telemetry.terminal_controller_intervened =
        result.terminal_controller_intervened;
    result.telemetry.terminal_spin_blocked =
        result.safety.terminal_spin_blocked;
    result.telemetry.tracking_safety_blocked =
        result.safety.tracking_safety_blocked;
    result.telemetry.safety_gate_intervened =
        result.telemetry.terminal_spin_blocked ||
        result.telemetry.tracking_safety_blocked;
    result.telemetry.phase_rejoin_evaluated = result.have_phase_decision;
    result.telemetry.phase_rejoin_recovery_used =
        result.have_phase_decision &&
        result.phase_decision.recovery_command_used;
    result.telemetry.phase_rejoin_controlled_stop_used =
        result.have_phase_decision &&
        result.phase_decision.controlled_stop_used;
    result.telemetry.solver_u0_a =
        result.output.first_shot_debug.u0_a;
    result.telemetry.solver_u0_alpha =
        result.output.first_shot_debug.u0_alpha;
    result.telemetry.planned_ax =
        result.output.first_shot_debug.u0_a;
    result.telemetry.planned_ay =
        result.solver_input.robot.v * result.solver_input.robot.omega;
    if (have_previous_shifted_plan_ &&
        previous_plan_cycle_id_ + 1 == request.cycle_id) {
        result.telemetry.previous_shifted_plan_available = true;
        result.telemetry.previous_plan_cycle_id = previous_plan_cycle_id_;
        result.telemetry.previous_shifted_plan_a = previous_shifted_plan_a_;
        result.telemetry.previous_shifted_plan_alpha =
            previous_shifted_plan_alpha_;
        result.telemetry.replanned_minus_shifted_a =
            result.telemetry.solver_u0_a - previous_shifted_plan_a_;
        result.telemetry.replanned_minus_shifted_alpha =
            result.telemetry.solver_u0_alpha - previous_shifted_plan_alpha_;
    }
    if (result.output.predicted_horizon.valid &&
        result.output.predicted_horizon.controls.size() > 1) {
        previous_plan_cycle_id_ = request.cycle_id;
        previous_shifted_plan_a_ =
            result.output.predicted_horizon.controls[1].a;
        previous_shifted_plan_alpha_ =
            result.output.predicted_horizon.controls[1].alpha_or_omega;
        have_previous_shifted_plan_ = true;
    } else {
        have_previous_shifted_plan_ = false;
    }
    result.phase_debug = makePhaseRejoinDebug(
        &result.phase_preparation,
        result.have_phase_decision ? &result.phase_decision : nullptr);
    if (result.safety.terminal_spin_blocked ||
        result.safety.tracking_safety_blocked) {
        result.phase_debug.status =
            "SAFETY_OVERRIDE_" + result.output.status;
    }
    result.telemetry.solver_command = result.solver_command;
    result.telemetry.terminal_command = result.terminal_command;
    result.telemetry.post_gate_command = result.decision.command;
    result.telemetry.final_command = result.decision.command;
    return result;
}

}  // namespace spmpc_local_planner
