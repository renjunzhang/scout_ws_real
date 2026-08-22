#pragma once

#include "spmpc_local_planner/domain/state.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_augmented_state.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace spmpc_local_planner {

enum class PhaseRejoinMode {
    Off = 0,
    Monitor = 1,
    Enforce = 2,
};

enum class PhaseRejoinEvidenceLevel {
    Unknown = 0,
    DevelopmentOnly = 1,
    EmpiricalHeldOut = 2,
};

struct EmpiricalRecoveryRadii {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double omega = 0.0;
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
};

// Per-phase hard box for the execution-only part of the augmented state:
// actuator outputs followed by the linear and angular pending-command queues.
// Bounds are empirical artifact data, never runtime defaults.
struct ExecutionCompatibilityBounds {
    bool valid = false;
    double linear_actuator_output = 0.0;
    double angular_actuator_output = 0.0;
    std::vector<double> linear_pending_commands;
    std::vector<double> angular_pending_commands;
};

struct PhaseNominalSample {
    std::size_t index = 0;
    double t = 0.0;
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double omega = 0.0;
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
    double a = 0.0;
    double alpha = 0.0;
    double v_s = 0.0;
    double u_pub_v = 0.0;
    double u_pub_omega = 0.0;
    double kappa_v = 0.0;
    double kappa_omega = 0.0;
    EmpiricalRecoveryRadii radii;
    bool augmented_execution_valid = false;
    ExecutionAugmentedState augmented_execution;
    ExecutionCompatibilityBounds execution_bounds;
};

struct NominalArtifactMetadata {
    std::string schema;
    PhaseRejoinEvidenceLevel evidence_level = PhaseRejoinEvidenceLevel::Unknown;
    std::string source;
    std::string contract_id;
    std::string frame_id;
    double dt = 0.0;
    double path_length = 0.0;
    // V2 artifacts make terminal ownership explicit.  Enforce mode is only
    // allowed to bypass the generic post-solver terminal clamp when this
    // contract has been parsed and independently checked by the loader.
    bool complete_terminal_tail = false;
    std::string terminal_contract;
    std::string recovery_contract;
    // V3 repeats the complete frozen recovery-policy image in its canonical
    // metadata.  The loader exact-matches these values against the compiled
    // bounded_tracking_recovery_policy_v1 implementation before admitting the
    // artifact.  V1/V2 leave them inactive and retain nominal_command_v1.
    double recovery_policy_longitudinal_position_gain = 0.0;
    double recovery_policy_lateral_position_gain = 0.0;
    double recovery_policy_yaw_gain = 0.0;
    double recovery_policy_linear_velocity_gain = 0.0;
    double recovery_policy_angular_velocity_gain = 0.0;
    double recovery_policy_max_residual_v = 0.0;
    double recovery_policy_max_residual_omega = 0.0;
    double recovery_policy_published_linear_min = 0.0;
    double recovery_policy_published_linear_max = 0.0;
    double recovery_policy_published_angular_min = 0.0;
    double recovery_policy_published_angular_max = 0.0;
    std::size_t terminal_zero_hold_steps = 0;
    double terminal_eta_norm_max = 0.0;
    double terminal_eta_dot_norm_max = 0.0;
    // V3 publish_zero_settle_hold_v2 keeps the published-command hold and
    // physical actuator settling as distinct contracts.  The legacy
    // stop_settle_zero_hold_v1 contract leaves these fields inactive.  A V3
    // generator selecting the v2 contract must serialize all six thresholds
    // below; the loader rejects a partial terminal contract.
    double terminal_v_abs_max = 0.0;
    double terminal_omega_abs_max = 0.0;
    double terminal_linear_actuator_output_abs_max = 0.0;
    double terminal_angular_actuator_output_abs_max = 0.0;
    double terminal_linear_pending_command_abs_max = 0.0;
    double terminal_angular_pending_command_abs_max = 0.0;
    // Continuous liquid model used to validate every discrete transition.
    double two_zeta_omega_n = 0.0;
    double omega_n_sq = 0.0;
    double kappa_x = 0.0;
    double kappa_y = 0.0;
    double dynamics_tolerance = 0.0;
    // V3 binds the complete augmented nominal and recovery interfaces used by
    // the nx=22 online solver.  V1/V2 leave these fields inactive and cannot
    // authorize that backend.
    bool delay_augmented_nominal = false;
    std::string execution_contract_id;
    std::string execution_contract_hash;
    int execution_state_width = 0;
    int linear_buffer_count = 0;
    int angular_buffer_count = 0;
    int parameter_schema_version = 0;
    std::string parameter_schema_id;
    std::string parameter_schema_hash;
    std::string recovery_artifact_hash;
    std::string execution_compatibility_contract;
};

// One stage of the phase-indexed nominal sequence passed into the solver.
// The gate fields are used only when gate_active=true.  A development/empirical
// gate must never be presented as a robust invariant set.
struct PhaseNominalStage {
    bool valid = false;
    bool gate_active = false;
    std::size_t artifact_index = 0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double s = 0.0;
    double v = 0.0;
    double omega = 0.0;
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
    double a = 0.0;
    double alpha = 0.0;
    double v_s = 0.0;
    double u_pub_v = 0.0;
    double u_pub_omega = 0.0;
    EmpiricalRecoveryRadii radii;
    bool augmented_execution_valid = false;
    ExecutionAugmentedState augmented_execution;
    ExecutionCompatibilityBounds execution_bounds;
};

struct DelayAugmentedPhaseCostWeights {
    double position = 0.0;
    double yaw = 0.0;
    double progress = 0.0;
    double v = 0.0;
    double omega = 0.0;
    double slosh_eta = 0.0;
    double slosh_eta_dot = 0.0;
    double linear_pending = 0.0;
    double angular_pending = 0.0;
    double acceleration = 0.0;
    double angular_acceleration = 0.0;
    double progress_rate = 0.0;
};

// Complete per-cycle parameter image consumed by the generated nx=22 solver.
// `stages` contains N+1 nominal augmented states; nominal controls at stage N
// are retained for a single canonical image even though terminal cost has no
// control argument.
struct DelayAugmentedPhaseSolverContext {
    bool active = false;
    int parameter_schema_version = 0;
    std::string parameter_schema_id;
    std::string parameter_schema_hash;
    std::string recovery_artifact_hash;
    std::string execution_compatibility_contract;
    int state_width = 0;
    int control_width = 0;
    int horizon_steps = 0;
    std::size_t current_index = 0;
    std::size_t terminal_index = 0;
    bool terminal_empirical_gate_bound = false;
    // C3 and C4 bind the same V3 radii and use the same generated 22D
    // capsule.  C4 enforces the terminal empirical inequality; strict C3
    // keeps the same metric as monitor-only evidence while disabling its
    // effect on the NLP and coordinator admission decision.
    bool terminal_empirical_gate_enforced = true;
    bool execution_compatibility_bound = false;
    double max_residual_v = 0.0;
    double max_residual_omega = 0.0;
    DelayAugmentedPhaseCostWeights weights;
    std::vector<PhaseNominalStage> stages;
};

struct PhaseRejoinSolverContext {
    bool active = false;
    // Only enforce contexts may alter the OCP.  Monitor contexts remain
    // available for diagnostics but are ignored by the solver wrapper.
    bool enforce = false;
    bool empirical_gate = false;
    bool state_complete_for_certificate = false;
    // True only for a loader-validated V2 sequence containing a dynamically
    // consistent slowdown, liquid-settling interval and zero-command hold.
    // This transfers terminal command ownership to the phase OCP; it does not
    // suppress the final GOAL_REACHED zero latch.
    bool owns_terminal_maneuver = false;
    // Set only after an accepted, command-consistent solve has validated the
    // final artifact window.  The generic GOAL_REACHED latch may run on the
    // following cycle; until then the phase controller retains tail ownership.
    bool terminal_release_authorized = false;
    std::size_t current_index = 0;
    std::size_t front_index = 0;
    std::size_t terminal_index = 0;
    int front_steps = 0;
    int liquid_steps = 0;
    double nominal_publish_v = 0.0;
    double nominal_publish_omega = 0.0;
    double max_residual_v = 0.0;
    double max_residual_omega = 0.0;
    std::vector<PhaseNominalStage> stages;
    // Present only for the explicit delay-augmented backend.  Legacy 10D
    // solvers ignore an inactive image and retain their frozen behavior.
    DelayAugmentedPhaseSolverContext delay_augmented;
};

// Single ownership predicate shared by SpmpcProblem and tests.  Returning true
// authorizes skipping only the generic pre-reached terminal clamp; safety gates
// and the reached zero latch retain their normal priority.
bool phaseRejoinOwnsTerminalCommand(
    const PhaseRejoinSolverContext& context);

struct PhaseCandidateSelectorParams {
    int backward_radius = 1;
    int forward_radius = 3;
    int initial_forward_radius = 4;
    // Candidate lead is measured against the absolute PhaseClock, never
    // against the last accepted candidate.  This prevents cumulative phase
    // acceleration even when forward_radius is configured too generously.
    int max_clock_lead_steps = 1;
    double weight_position = 1.0;
    double weight_yaw = 0.5;
    double weight_velocity = 0.5;
    double weight_liquid = 1.0;
};

struct PhaseRejoinParams {
    PhaseRejoinMode mode = PhaseRejoinMode::Off;
    // This is the only formal C3/C4 ablation switch.  It never disables
    // B_exec, phase selection, residual authority, recovery action, or the
    // final-command transaction.
    bool empirical_gate_enforced = true;
    PhaseCandidateSelectorParams candidate;
    int liquid_horizon_steps = 3;
    double max_residual_v = 0.08;
    double max_residual_omega = 0.20;
    double artifact_dt_tolerance_sec = 1e-4;
    double artifact_path_length_tolerance_m = 0.05;
    double artifact_path_geometry_tolerance_m = 0.075;
    // Normalized coefficient tolerance: |a-b| <= tol*max(1,|b|).
    double artifact_model_tolerance = 1e-6;
    double artifact_command_tolerance = 1e-8;
    bool allow_development_artifact_in_enforce = false;
    std::string required_contract_id;
    std::string required_frame_id;
};

// Values derived from the actual runtime planner configuration.  Keeping
// these in one explicit object prevents an artifact from being validated
// against only a path length while the solver uses different geometry,
// liquid dynamics, or command bounds.
struct PhaseRejoinRuntimeContract {
    bool liquid_model_configured = false;
    double dt = 0.0;
    double two_zeta_omega_n = 0.0;
    double omega_n_sq = 0.0;
    double kappa_x = 1.0;
    double kappa_y = 1.0;
    double min_command_v = 0.0;
    double max_command_v = 0.0;
    double max_abs_command_omega = 0.0;
    bool delay_augmented_solver_requested = false;
    std::string execution_contract_id;
    std::string execution_contract_hash;
    int execution_state_width = 0;
    int linear_buffer_count = 0;
    int angular_buffer_count = 0;
    int solver_control_width = 0;
    int execution_front_steps = 0;
    int solver_horizon_steps = 0;
    // Hard rates of the newly published command used by the generated
    // augmented OCP.  Recovery actions bypass the optimizer, so the
    // coordinator independently reapplies these bounds against the trusted
    // current pending-command tail before publication.
    double max_published_acceleration = 0.0;
    double max_published_angular_acceleration = 0.0;
    int parameter_schema_version = 0;
    std::string parameter_schema_id;
    std::string parameter_schema_hash;
    std::string recovery_artifact_hash;
    std::string execution_compatibility_contract;
    std::uint32_t solver_capabilities = 0;
    std::uint32_t required_solver_capabilities = 0;
    DelayAugmentedPhaseCostWeights delay_augmented_weights;
};

struct PhaseCandidateResult {
    bool valid = false;
    std::size_t clock_index = 0;
    std::size_t current_index = 0;
    std::size_t front_index = 0;
    std::size_t terminal_index = 0;
    std::size_t normal_shift_index = 0;
    std::size_t candidate_window_begin_index = 0;
    std::size_t candidate_window_end_index = 0;
    std::size_t candidate_count = 0;
    bool execution_compatibility_filter_applied = false;
    std::size_t execution_rejected_candidate_count = 0;
    int phase_lead_steps = 0;
    double score = 0.0;
    double selected_execution_max_normalized_error = 0.0;
    struct ExecutionCandidateAudit {
        std::size_t phase_index = 0;
        bool valid = false;
        bool accepted = false;
        double max_normalized_error = 0.0;
        std::string max_error_name = "NONE";
        int max_error_index = -1;
        double actual = 0.0;
        double nominal = 0.0;
        double bound = 0.0;
        std::string status = "NOT_RUN";
    };
    std::vector<ExecutionCandidateAudit> execution_candidate_audits;
    std::string status = "NOT_RUN";
};

struct EmpiricalRecoveryGateResult {
    bool valid = false;
    bool accepted = false;
    double metric = 0.0;
    double max_normalized_error = 0.0;
    std::string status = "NOT_RUN";
};

struct ExecutionCompatibilityGateResult {
    bool valid = false;
    bool accepted = false;
    double max_normalized_error = 0.0;
    std::string max_error_name = "NONE";
    int max_error_index = -1;
    double actual = 0.0;
    double nominal = 0.0;
    double bound = 0.0;
    std::string status = "NOT_RUN";
};

struct PhaseRejoinPreparation {
    bool ready = false;
    bool command_intervention_allowed = false;
    PhaseCandidateResult candidate;
    PhaseRejoinSolverContext solver_context;
    double nominal_cmd_v = 0.0;
    double nominal_cmd_omega = 0.0;
    double recovery_cmd_v = 0.0;
    double recovery_cmd_omega = 0.0;
    int solver_terminal_step = 0;
    bool solver_origin_at_execution_front = true;
    bool solver_origin_is_execution_augmented = false;
    double phase_clock_elapsed_sec = 0.0;
    std::string status = "NOT_RUN";
};

// Narrow, allocation-free view of the solver result consumed by Phase-Rejoin.
// The controller owns conversion from backend/telemetry horizon DTOs so this
// module depends only on domain state and never on the full SolverOutput.
struct PhaseSolveView {
    double cmd_v = 0.0;
    double cmd_omega = 0.0;
    bool optimization_failure_recovery_eligible = false;
    bool terminal_state_available = false;
    RobotState terminal_robot;
    SloshState terminal_slosh;
    bool current_execution_state_available = false;
    ExecutionAugmentedState current_execution;
    bool terminal_execution_state_available = false;
    ExecutionAugmentedState terminal_execution;
};

struct PhaseRejoinDecision {
    bool evaluated = false;
    bool terminal_gate_accepted = false;
    bool current_gate_accepted = false;
    bool terminal_execution_compatible = false;
    bool current_execution_compatible = false;
    bool command_intervened = false;
    bool recovery_command_used = false;
    bool controlled_stop_used = false;
    bool command_contract_consistent = false;
    double solver_cmd_v = 0.0;
    double solver_cmd_omega = 0.0;
    double output_cmd_v = 0.0;
    double output_cmd_omega = 0.0;
    double residual_v = 0.0;
    double residual_omega = 0.0;
    EmpiricalRecoveryGateResult terminal_gate;
    EmpiricalRecoveryGateResult current_gate;
    ExecutionCompatibilityGateResult terminal_execution_gate;
    ExecutionCompatibilityGateResult current_execution_gate;
    std::string status = "NOT_RUN";
};

// ROS-independent payload.  The distinct name avoids colliding with the
// generated spmpc_local_planner/PhaseRejoinDebug message type.
struct PhaseRejoinDebugData {
    PhaseRejoinMode mode = PhaseRejoinMode::Off;
    PhaseRejoinEvidenceLevel evidence_level = PhaseRejoinEvidenceLevel::Unknown;
    bool artifact_loaded = false;
    bool contract_valid = false;
    bool ready = false;
    bool empirical_gate = false;
    bool empirical_gate_enforced = false;
    bool state_complete_for_certificate = false;
    std::size_t artifact_size = 0;
    std::size_t current_index = 0;
    std::size_t clock_index = 0;
    std::size_t front_index = 0;
    std::size_t terminal_index = 0;
    std::size_t candidate_count = 0;
    int phase_lead_steps = 0;
    int front_steps = 0;
    int liquid_steps = 0;
    int solver_terminal_step = 0;
    bool solver_origin_at_execution_front = true;
    double candidate_score = 0.0;
    double terminal_gate_metric = 0.0;
    double current_gate_metric = 0.0;
    bool terminal_gate_accepted = false;
    bool current_gate_accepted = false;
    bool command_intervened = false;
    bool recovery_command_used = false;
    bool controlled_stop_used = false;
    bool command_contract_consistent = false;
    bool terminal_release_authorized = false;
    double nominal_cmd_v = 0.0;
    double nominal_cmd_omega = 0.0;
    double solver_cmd_v = 0.0;
    double solver_cmd_omega = 0.0;
    double output_cmd_v = 0.0;
    double output_cmd_omega = 0.0;
    double residual_v = 0.0;
    double residual_omega = 0.0;
    std::string contract_id;
    std::string artifact_path;
    std::string status = "OFF";
};

std::string phaseRejoinModeName(PhaseRejoinMode mode);
bool parsePhaseRejoinMode(const std::string& text, PhaseRejoinMode& mode);
std::string phaseRejoinEvidenceLevelName(PhaseRejoinEvidenceLevel level);
bool parsePhaseRejoinEvidenceLevel(const std::string& text,
                                   PhaseRejoinEvidenceLevel& level);

}  // namespace spmpc_local_planner
