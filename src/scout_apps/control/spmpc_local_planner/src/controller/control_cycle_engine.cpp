#include "spmpc_local_planner/controller/control_cycle_engine.h"
#include "spmpc_local_planner/controller/phase_solve_adapter.h"

#include <cmath>

namespace spmpc_local_planner {
namespace {

bool backupTailEligiblePreparationStatus(const std::string& status) {
    return status == "NO_EXECUTION_COMPATIBLE_CANDIDATE" ||
        status == "NO_EXECUTION_HORIZON_COMPATIBLE_CANDIDATE" ||
        status == "NOMINAL_HORIZON_INCOMPLETE" ||
        status == "AUGMENTED_NOMINAL_HORIZON_INCOMPLETE" ||
        status == "SOLVER_HORIZON_TOO_SHORT" ||
        status == "DELAY_AUGMENTED_HORIZON_MISMATCH";
}

}  // namespace

ControlCycleEngine::ControlCycleEngine(SolverSession& solver_session)
    : solver_session_(solver_session),
      publication_transaction_(command_pipeline_) {}

bool ControlCycleEngine::configurePhaseRejoin(
    const PhaseRejoinParams& params,
    std::string& error) {
    if (!phase_rejoin_.configure(params, error)) {
        return false;
    }
    tail_commit_.configure(params.tail_commit_enabled);
    phase_params_ = params;
    goal_reached_latched_ = false;
    return true;
}

NominalArtifactLoadResult ControlCycleEngine::loadPhaseRejoinArtifact(
    const std::string& path) {
    const NominalArtifactLoadResult result =
        phase_rejoin_.loadArtifact(path);
    if (result.success &&
        !tail_commit_.setArtifactSize(
            phase_rejoin_.artifact().size())) {
        NominalArtifactLoadResult failed = result;
        failed.success = false;
        failed.status = "TAIL_COMMIT_ARTIFACT_SIZE_INVALID";
        failed.detail = "loaded phase artifact is empty";
        return failed;
    }
    return result;
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

bool ControlCycleEngine::configurePublishLatency(
    const PublishLatencyModelConfig& config,
    std::string& error) {
    return publish_latency_model_.configure(config, error);
}

PublishEpochEstimate ControlCycleEngine::estimatePublishEpoch(
    const CycleTimingContract& cycle) const {
    return publish_latency_model_.estimate(cycle);
}

SpeedReferenceConfigureResult ControlCycleEngine::configureSpeedReference(
    const SpeedReferenceControllerConfig& config) {
    return speed_reference_.configure(config);
}

SpeedReferenceEvaluation ControlCycleEngine::prepareSpeedReference(
    SolverInput& input) {
    return speed_reference_.apply(input.robot, input.slosh, input);
}

CommandPublicationResult ControlCycleEngine::publishDecision(
    const PublishEpochEstimate& publish_epoch_estimate,
    const CommandDecision& decision,
    bool force_zero,
    bool publish_enabled,
    ICommandSink* sink,
    CommandHistoryBuffer* history) {
    CommandPublicationRequest request;
    request.cycle_id = publish_epoch_estimate.cycle.cycle_id;
    request.proposed = decision;
    request.force_zero = force_zero;
    request.publish_enabled = publish_enabled;
    request.sink = sink;
    request.history = history;
    CommandPublicationResult result =
        publication_transaction_.execute(request);
    result.publish_timing = publish_latency_model_.observe(
        publish_epoch_estimate,
        result.receipt.delivered
            ? result.receipt.actual_publish_stamp_ns
            : 0);
    return result;
}

CommandPublicationResult ControlCycleEngine::publishFailClosedZero(
    std::uint64_t cycle_id,
    StampNs cycle_start_ns,
    double control_period_sec,
    ICommandSink* sink,
    CommandHistoryBuffer* history,
    bool publish_enabled,
    const std::string& reason) {
    CommandDecision decision;
    decision.source = CommandSource::FailClosed;
    decision.reason = reason.empty() ? "FAIL_CLOSED_ZERO" : reason;
    decision.accepted = false;
    CycleTimingContract timing;
    timing.cycle_id = cycle_id;
    timing.cycle_start_stamp_ns = cycle_start_ns;
    timing.control_period_sec = control_period_sec;
    return publishDecision(
        publish_latency_model_.estimate(timing),
        decision, true, publish_enabled, sink, history);
}

void ControlCycleEngine::resetSafety() {
    safety_.reset();
}

void ControlCycleEngine::resetForReference() {
    safety_.reset();
    phase_rejoin_.resetProgress();
    tail_commit_.reset();
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
    const bool execution_augmented =
        request.solver_input.execution_horizon.active;
    if (tail_commit_.state() == TailCommitState::Released) {
        preparation.ready = true;
        preparation.tail_released = true;
        preparation.status = "TAIL_RELEASED";
        return preparation;
    }
    if (tail_commit_.state() == TailCommitState::Aborted) {
        preparation.tail_aborted = true;
        preparation.status = tail_commit_.reason();
        return preparation;
    }
    if (tail_commit_.state() == TailCommitState::Committed) {
        if (!execution_augmented ||
            !request.solver_input.execution_horizon.initial_state.valid) {
            preparation.tail_committed_mode = true;
            preparation.tail_artifact_index = tail_commit_.cursor();
            preparation.status = "TAIL_EXECUTION_STATE_UNAVAILABLE";
            return preparation;
        }
        return phase_rejoin_.prepareCommittedTail(
            tail_commit_.cursor(),
            request.solver_input.execution_horizon.initial_state);
    }
    if (phase_params_.mode == PhaseRejoinMode::Off) {
        preparation.status = "OFF";
    } else if (!execution_augmented && !request.prediction_valid) {
        preparation.status = "PREDICTION_" + request.prediction_status;
    } else if (phase_params_.mode == PhaseRejoinMode::Enforce &&
               !request.solver_origin_at_execution_front &&
               !execution_augmented) {
        preparation.status = "EXECUTION_FRONT_NOT_APPLIED";
    } else {
        const RobotState& phase_robot = execution_augmented
            ? request.solver_input.execution_horizon.initial_state.robot
            : request.execution_front_robot;
        const SloshState& phase_slosh = execution_augmented
            ? request.solver_input.execution_horizon.initial_state.slosh
            : request.execution_front_slosh;
        const int front_steps = execution_augmented
            ? request.solver_input.execution_horizon.execution_front_steps
            : request.execution_front_steps;
        const int solver_horizon = execution_augmented
            ? request.solver_input.execution_horizon.horizon_steps
            : request.solver_input.horizon_steps;
        const double phase_time_sec = execution_augmented
            ? static_cast<double>(
                  request.solver_input.execution_horizon.initial_epoch_ns) *
                  kSecondsPerNanosecond
            : request.phase_time_sec;
        preparation = phase_rejoin_.prepare(
            phase_robot,
            phase_slosh,
            front_steps,
            solver_horizon,
            phase_time_sec,
            request.solver_origin_at_execution_front,
            execution_augmented,
            execution_augmented
                ? &request.solver_input.execution_horizon.initial_state
                : nullptr,
            execution_augmented
                ? &request.solver_input.execution_horizon
                : nullptr);
    }
    if (!preparation.ready &&
        backupTailEligiblePreparationStatus(preparation.status)) {
        const std::size_t phase_clock_index =
            preparation.phase_clock_index;
        PhaseRejoinPreparation backup = prepareBackupTail(request);
        if (backup.ready) {
            backup.phase_clock_index = phase_clock_index;
            preparation = backup;
        }
    }
    return preparation;
}

PhaseRejoinPreparation ControlCycleEngine::prepareBackupTail(
    const ControlCycleRequest& request) const {
    PhaseRejoinPreparation preparation;
    if (!phase_rejoin_.haveAcceptedIndex()) {
        preparation.status = "TAIL_BACKUP_UNAVAILABLE_NO_ACCEPTED_INDEX";
        return preparation;
    }
    if (!request.solver_input.execution_horizon.active ||
        !request.solver_input.execution_horizon.initial_state.valid) {
        preparation.status = "TAIL_BACKUP_EXECUTION_STATE_UNAVAILABLE";
        return preparation;
    }
    preparation = phase_rejoin_.prepareCommittedTail(
        phase_rejoin_.acceptedIndex(),
        request.solver_input.execution_horizon.initial_state);
    if (preparation.ready) {
        preparation.tail_backup_used = true;
        preparation.status = "TAIL_BACKUP_READY";
    }
    return preparation;
}

ControlCycleResult ControlCycleEngine::step(
    const ControlCycleRequest& request) {
    ControlCycleResult result;
    PublishEpochEstimate publish_epoch_estimate =
        request.publish_epoch_estimate;
    CycleTimingContract timing;
    timing.cycle_id = request.cycle_id;
    timing.cycle_start_stamp_ns = request.cycle_start_ns;
    timing.control_period_sec = request.control_period_sec;
    if (!publishEpochEstimateMatchesCycle(
            publish_epoch_estimate, timing)) {
        publish_epoch_estimate = publish_latency_model_.estimate(timing);
    }
    result.solver_input = request.solver_input;
    result.solver_input.publish_epoch_estimate =
        publish_epoch_estimate;
    applyPublishEpochEstimate(
        publish_epoch_estimate,
        result.solver_input.cycle_timing);
    const bool execution_epoch_mismatch =
        result.solver_input.execution_horizon.active &&
        (!publish_epoch_estimate.valid ||
         publish_epoch_estimate.expected_deadline_missed ||
         !validStamp(
             result.solver_input.execution_horizon.initial_epoch_ns) ||
         result.solver_input.execution_horizon.initial_epoch_ns !=
             publish_epoch_estimate.expected_publish_stamp_ns);
    const bool tail_history_missing =
        phase_params_.tail_commit_enabled &&
        request.command_history == nullptr;
    bool solver_invocation_allowed =
        !execution_epoch_mismatch && !tail_history_missing;
    if (execution_epoch_mismatch) {
        // The augmented state is meaningful only at the publication epoch it
        // was aligned to.  Recomputing cycle timing must never silently reuse
        // a state/buffer image from another epoch or invoke a recovery action.
        result.phase_preparation.status =
            "EXECUTION_HORIZON_PUBLISH_EPOCH_MISMATCH";
        result.solver_input.phase_rejoin = PhaseRejoinSolverContext{};
        result.solver_output.status =
            "EXECUTION_HORIZON_PUBLISH_EPOCH_MISMATCH";
    } else if (tail_history_missing) {
        result.phase_preparation.status =
            "TAIL_COMMAND_HISTORY_REQUIRED";
        result.solver_input.phase_rejoin = PhaseRejoinSolverContext{};
        result.solver_output.status =
            "TAIL_COMMAND_HISTORY_REQUIRED";
    } else {
        ControlCycleRequest normalized_request = request;
        normalized_request.publish_epoch_estimate =
            publish_epoch_estimate;
        normalized_request.solver_input = result.solver_input;
        result.phase_preparation = preparePhase(normalized_request);
        result.solver_input.phase_rejoin =
            result.phase_preparation.solver_context;
        const bool tail_direct_command =
            result.phase_preparation.tail_committed_mode ||
            result.phase_preparation.tail_commit_armed;
        const bool no_execution_compatible_candidate =
            result.solver_input.execution_horizon.active &&
            (result.phase_preparation.status ==
                 "NO_EXECUTION_COMPATIBLE_CANDIDATE" ||
             result.phase_preparation.status ==
                 "NO_EXECUTION_HORIZON_COMPATIBLE_CANDIDATE");
        if (result.phase_preparation.tail_released) {
            solver_invocation_allowed = false;
            result.solver_output.success = true;
            result.solver_output.status = "GOAL_REACHED";
            result.solver_output.terminal_diagnostics.reached = true;
        } else if (result.phase_preparation.tail_aborted) {
            solver_invocation_allowed = false;
            result.solver_output.status =
                result.phase_preparation.status;
        } else if (tail_direct_command) {
            solver_invocation_allowed = false;
            result.solver_output.status =
                result.phase_preparation.tail_committed_mode
                ? "NOT_RUN_TAIL_COMMITTED"
                : "NOT_RUN_TAIL_ARMED";
        } else if (no_execution_compatible_candidate) {
            solver_invocation_allowed = false;
            result.solver_output.status =
                "NOT_RUN_" + result.phase_preparation.status;
        } else {
            result.solve_returned = solver_session_.solve(
                result.solver_input, result.solver_output);
            if (!result.solver_output.success &&
                result.solver_output.failure_kind ==
                    SolverFailureKind::Integrity &&
                phase_rejoin_.haveAcceptedIndex()) {
                PhaseRejoinPreparation backup =
                    prepareBackupTail(normalized_request);
                if (backup.ready) {
                    backup.phase_clock_index =
                        result.phase_preparation.phase_clock_index;
                    result.phase_preparation = backup;
                    result.solver_input.phase_rejoin =
                        result.phase_preparation.solver_context;
                }
            }
        }
    }
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
    if (!execution_epoch_mismatch &&
        phase_params_.mode != PhaseRejoinMode::Off) {
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
                    result.phase_preparation.solver_terminal_step,
                    request.solver_input.execution_horizon.active
                        ? &request.solver_input.execution_horizon.initial_state
                        : nullptr));
            if (phase_params_.mode == PhaseRejoinMode::Enforce) {
                result.output.cmd_v =
                    result.phase_decision.output_cmd_v;
                result.output.cmd_omega =
                    result.phase_decision.output_cmd_omega;
                result.output.status = result.phase_decision.status;
                const bool terminal_empirical_admitted =
                    !phase_params_.empirical_gate_enforced ||
                    result.phase_decision.terminal_gate_accepted;
                result.output.success =
                    (result.solver_success &&
                     terminal_empirical_admitted &&
                     result.phase_decision.current_execution_compatible &&
                     result.phase_decision.terminal_execution_compatible &&
                     result.phase_decision.command_contract_consistent) ||
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
    // Every decision after execution-horizon construction must use one epoch.
    // The augmented solver, Phase-Rejoin gate and path projection all start at
    // the expected publication epoch, so feeding the older source-epoch robot
    // state to the spin latch could either miss pending rotation or reject a
    // rotation that the aligned execution state has already settled.
    safety_input.robot =
        result.solver_input.execution_horizon.active &&
            result.solver_input.execution_horizon.initial_state.valid
        ? result.solver_input.execution_horizon.initial_state.robot
        : result.solver_input.robot;
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

    const bool phase_commit_candidate =
        result.have_phase_decision && !result.terminal_priority &&
        !result.safety.blocked &&
        (phase_params_.mode == PhaseRejoinMode::Monitor ||
         result.output.success);

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
        !result.safety.blocked;
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
    result.publication = publishDecision(
        publish_epoch_estimate,
        result.decision,
        !result.decision.accepted,
        request.publish_enabled,
        request.command_sink,
        request.command_history);
    result.final_command = result.publication.pipeline.final_command;
    result.output.cmd_v = result.final_command.linear;
    result.output.cmd_omega = result.final_command.angular;
    result.output.success = result.publication.pipeline.decision.accepted;
    if (result.publication.pipeline.decision.source ==
        CommandSource::ExecutionContract) {
        result.output.status = result.publication.pipeline.decision.reason;
    }

    const bool tail_lifecycle_active =
        phase_params_.tail_commit_enabled &&
        (result.phase_preparation.tail_commit_armed ||
         result.phase_preparation.tail_committed_mode ||
         result.phase_decision.tail_commit_requested ||
         result.phase_decision.tail_committed_mode ||
         tail_commit_.state() == TailCommitState::Committed);
    if (tail_lifecycle_active) {
        TailPublicationEvidence evidence;
        evidence.artifact_index = result.phase_decision.tail_command_used
            ? result.phase_decision.tail_artifact_index
            : tail_commit_.cursor();
        evidence.tail_command = result.phase_decision.tail_command_used;
        evidence.delivered = result.publication.published();
        evidence.receipt_consistent =
            result.publication.receipt_consistent;
        evidence.command_unmodified =
            !result.publication.commandWasModified() &&
            result.decision.source == CommandSource::PhaseRejoin;
        evidence.safety_override = result.safety.blocked;
        evidence.contract_valid = !execution_epoch_mismatch &&
            result.phase_decision.command_contract_consistent &&
            !result.phase_decision.controlled_stop_used &&
            result.publication.limiter_state_committed &&
            result.publication.history_committed;
        result.tail_commit = tail_commit_.onPublication(evidence);
        result.tail_publication_observed = true;
        if (result.tail_commit.state == TailCommitState::Released) {
            phase_rejoin_.authorizeTailRelease();
        }
    } else if (phase_commit_candidate &&
               result.publication.published() &&
               !result.publication.commandWasModified()) {
        result.phase_committed = phase_rejoin_.commit(
            result.phase_preparation, result.phase_decision);
    }

    speed_reference_.commitProgress(result.output.progress_abs_s);

    // Freeze the complete controller-owned decision trace before returning to
    // any transport adapter.  ROS may add observer/publish timing, but it no
    // longer needs to infer solver, terminal, phase or safety interventions.
    result.telemetry.cycle_id = request.cycle_id;
    result.telemetry.cycle_start_ns = request.cycle_start_ns;
    result.telemetry.status = result.output.status;
    result.telemetry.solver_status = result.solver_output.status;
    result.telemetry.command_reason =
        result.publication.pipeline.decision.reason;
    result.telemetry.command_source =
        result.publication.pipeline.decision.source;
    result.telemetry.solve_attempted = solver_invocation_allowed;
    result.telemetry.solve_returned = result.solve_returned;
    result.telemetry.solve_success = result.solver_success;
    result.telemetry.command_accepted =
        result.publication.pipeline.decision.accepted;
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
    result.telemetry.phase_rejoin_committed = result.phase_committed;
    result.telemetry.publication_attempted =
        result.publication.receipt.attempted;
    result.telemetry.command_was_published =
        result.publication.receipt.delivered;
    result.telemetry.publication_receipt_consistent =
        result.publication.receipt_consistent;
    result.telemetry.command_history_committed =
        result.publication.history_committed;
    result.telemetry.command_publish_stamp_ns =
        result.publication.receipt.actual_publish_stamp_ns;
    result.telemetry.publish_timing = result.publication.publish_timing;
    result.telemetry.command_contract_violation =
        result.publication.pipeline.command_contract_violation ||
        result.publication.pipeline.finite_violation;
    result.telemetry.linear_limited =
        result.publication.pipeline.linear_limited;
    result.telemetry.angular_rate_limited =
        result.publication.pipeline.angular_rate_limited;
    result.telemetry.angular_accel_limited =
        result.publication.pipeline.angular_accel_limited;
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
    result.telemetry.final_command = result.final_command;
    if (result.publication.receipt.delivered) {
        result.telemetry.published_command =
            result.publication.receipt.command;
    }
    return result;
}

}  // namespace spmpc_local_planner
