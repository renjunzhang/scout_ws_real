#pragma once

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
    std::size_t terminal_zero_hold_steps = 0;
    double terminal_eta_norm_max = 0.0;
    double terminal_eta_dot_norm_max = 0.0;
    // Continuous liquid model used to validate every discrete transition.
    double two_zeta_omega_n = 0.0;
    double omega_n_sq = 0.0;
    double kappa_x = 0.0;
    double kappa_y = 0.0;
    double dynamics_tolerance = 0.0;
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
    EmpiricalRecoveryRadii radii;
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
};

struct PhaseCandidateResult {
    bool valid = false;
    std::size_t clock_index = 0;
    std::size_t current_index = 0;
    std::size_t front_index = 0;
    std::size_t terminal_index = 0;
    std::size_t normal_shift_index = 0;
    std::size_t candidate_count = 0;
    int phase_lead_steps = 0;
    double score = 0.0;
    std::string status = "NOT_RUN";
};

struct EmpiricalRecoveryGateResult {
    bool valid = false;
    bool accepted = false;
    double metric = 0.0;
    double max_normalized_error = 0.0;
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
    double phase_clock_elapsed_sec = 0.0;
    std::string status = "NOT_RUN";
};

struct PhaseRejoinDecision {
    bool evaluated = false;
    bool terminal_gate_accepted = false;
    bool current_gate_accepted = false;
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
