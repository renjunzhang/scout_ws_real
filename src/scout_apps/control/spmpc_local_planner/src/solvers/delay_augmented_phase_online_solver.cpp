#include "spmpc_local_planner/solvers/delay_augmented_phase_online_solver.h"

#include "spmpc_local_planner/reference/progress_projector.h"
#include "spmpc_local_planner/solver/api/backend.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

bool same(double lhs, double rhs) {
    return std::isfinite(lhs) && std::isfinite(rhs) && lhs == rhs;
}

bool lowercaseSha256(const std::string& value) {
    return value.size() == 64 &&
        std::all_of(value.begin(), value.end(), [](char character) {
            return (character >= '0' && character <= '9') ||
                (character >= 'a' && character <= 'f');
        });
}

bool sameSlosh(const SloshModelParams& actual,
               const SloshModelParams& expected) {
    return same(actual.dt, expected.dt) &&
        same(actual.container_radius, expected.container_radius) &&
        same(actual.liquid_height, expected.liquid_height) &&
        same(actual.liquid_density, expected.liquid_density) &&
        same(actual.damping_ratio, expected.damping_ratio) &&
        actual.mode_index == expected.mode_index &&
        same(actual.slosh_height_ref, expected.slosh_height_ref) &&
        same(actual.slosh_eta_dot_ratio,
             expected.slosh_eta_dot_ratio) &&
        actual.use_linear_model == expected.use_linear_model &&
        actual.use_parabola_term == expected.use_parabola_term;
}

bool finiteWeights(const VariantConfig& variant) {
    const double values[] = {
        variant.w_contour, variant.w_lag, variant.w_progress,
        variant.w_v, variant.w_control, variant.w_slosh,
        variant.w_accel, variant.w_alpha, variant.w_vs,
        variant.w_control + variant.w_accel,
    };
    return std::all_of(std::begin(values), std::end(values),
                       [](double value) {
                           return std::isfinite(value) && value >= 0.0;
                       });
}

bool weightsMatchRuntimeConfig(
    const DelayAugmentedPhaseCostWeights& actual,
    const VariantConfig& variant,
    const SloshModelParams& slosh) {
    return same(actual.position, variant.w_contour) &&
        same(actual.yaw, variant.w_lag) &&
        same(actual.progress, variant.w_progress) &&
        same(actual.v, variant.w_v) &&
        same(actual.omega, variant.w_control) &&
        same(actual.slosh_eta, variant.w_slosh) &&
        same(actual.slosh_eta_dot,
             variant.w_slosh * slosh.slosh_eta_dot_ratio) &&
        same(actual.linear_pending, variant.w_v) &&
        same(actual.angular_pending, variant.w_control) &&
        same(actual.acceleration,
             variant.w_control + variant.w_accel) &&
        same(actual.angular_acceleration, variant.w_alpha) &&
        same(actual.progress_rate, variant.w_vs);
}

HorizonStateDebug horizonState(const DelayAugmentedPhaseState& state) {
    HorizonStateDebug point;
    point.x = state.execution.robot.x;
    point.y = state.execution.robot.y;
    point.yaw = state.execution.robot.yaw;
    point.v = state.execution.robot.v;
    point.s = state.progress_s;
    point.omega = state.execution.robot.omega;
    point.eta_x = state.execution.slosh.eta_x;
    point.eta_x_dot = state.execution.slosh.eta_x_dot;
    point.eta_y = state.execution.slosh.eta_y;
    point.eta_y_dot = state.execution.slosh.eta_y_dot;
    point.h_modal = manifest::kSloshHeightRef /
        manifest::kEtaScale * std::hypot(point.eta_x, point.eta_y);
    return point;
}

bool phaseResidualsAdmissible(
    const DelayAugmentedPhaseSolverContext& context,
    const DelayAugmentedPhaseRolloutResult& rollout,
    std::string& error) {
    if (!rollout.valid || rollout.controls.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps) ||
        rollout.published_commands.size() != rollout.controls.size() ||
        context.stages.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps + 1)) {
        error = "causal phase horizon cardinality mismatch";
        return false;
    }
    const double tolerance = manifest::kMaxInequalityResidual;
    for (std::size_t stage = 0; stage < rollout.controls.size(); ++stage) {
        const DelayAugmentedPhaseControl& control = rollout.controls[stage];
        const VelocityCommand& published = rollout.published_commands[stage];
        const PhaseNominalStage& nominal = context.stages[stage];
        if (!std::isfinite(control.acceleration) ||
            !std::isfinite(control.angular_acceleration) ||
            !std::isfinite(control.progress_rate) ||
            std::abs(published.linear - nominal.u_pub_v) >
                context.max_residual_v + tolerance ||
            std::abs(published.angular - nominal.u_pub_omega) >
                context.max_residual_omega + tolerance) {
            error = "causal published residual rejected at stage " +
                std::to_string(stage);
            return false;
        }
    }
    return true;
}

class BackendWallTimer {
public:
    explicit BackendWallTimer(SolverOutput& output)
        : output_(output), begin_(std::chrono::steady_clock::now()) {}
    ~BackendWallTimer() {
        const auto elapsed = std::chrono::steady_clock::now() - begin_;
        output_.pre_solve_snapshot.backend_wall_time_ms =
            std::chrono::duration<double, std::milli>(elapsed).count();
    }

private:
    SolverOutput& output_;
    std::chrono::steady_clock::time_point begin_;
};

}  // namespace

SolverConfigureResult DelayAugmentedPhaseOnlineSolver::configure(
    const SolverParams& params,
    const VariantConfig& variant) {
    configured_ = false;
    SolverConfigureResult result;
    const DelayAugmentedPhaseCompiledContract compiled =
        DelayAugmentedPhaseAcadosSolver::compiledContract();
    const DelayAugmentedPhaseBackendParams& requested =
        params.delay_augmented_phase;
    if (params.solver_backend !=
            kSolverBackendDelayAugmentedPhaseAcados ||
        !requested.enabled) {
        result.status = "DELAY_AUGMENTED_BACKEND_NOT_EXPLICITLY_ENABLED";
        return result;
    }
    if (!DelayAugmentedPhaseAcadosSolver::compiled()) {
        result.status = "DELAY_AUGMENTED_CAPSULE_NOT_COMPILED";
        return result;
    }
    if (!lowercaseSha256(
            requested.expected_recovery_artifact_hash)) {
        result.status =
            "DELAY_AUGMENTED_RECOVERY_ASSET_NOT_FROZEN";
        return result;
    }
    if (requested.execution_contract_id !=
            compiled.execution.contract_id ||
        requested.execution_contract_hash !=
            compiled.execution.contract_hash ||
        requested.expected_state_width != compiled.state_width ||
        requested.expected_control_width != compiled.control_width ||
        requested.expected_horizon_steps != compiled.horizon_steps ||
        requested.parameter_schema_version !=
            compiled.parameter_schema_version ||
        requested.parameter_schema_id != compiled.parameter_schema_id ||
        requested.parameter_schema_hash != compiled.parameter_schema_hash ||
        requested.required_capabilities !=
            kDelayAugmentedPhaseFormalCapabilities ||
        (compiled.capabilities & requested.required_capabilities) !=
            requested.required_capabilities) {
        result.status = "DELAY_AUGMENTED_COMPILED_CONTRACT_MISMATCH";
        return result;
    }
    if (!variant.slosh_enable || !finiteWeights(variant) ||
        !same(params.v_max, compiled.execution.linear.output_max) ||
        !same(params.omega_max, compiled.execution.angular.output_max) ||
        !same(params.a_max, compiled.acceleration_max) ||
        !same(params.alpha_max, compiled.angular_acceleration_max) ||
        !sameSlosh(params.slosh, compiled.slosh)) {
        result.status = "DELAY_AUGMENTED_RUNTIME_MODEL_MISMATCH";
        return result;
    }
    params_ = params;
    variant_ = variant;
    configured_ = true;
    result.success = true;
    result.status = "OK";
    result.detail = "nx=22 nu=3 N=10 parameter_hash=" +
        compiled.parameter_schema_hash;
    return result;
}

bool DelayAugmentedPhaseOnlineSolver::solve(
    const SolverInput& input,
    const ReferencePath& reference,
    SolverOutput& output) {
    output = SolverOutput{};
    BackendWallTimer backend_wall_timer(output);
    // Every early return in this explicit backend is a contract/integrity
    // rejection unless the numerical optimizer itself is reached and fails.
    // Phase-Rejoin may use stored recovery only for that latter class.
    output.failure_kind = SolverFailureKind::Integrity;
    output.cycle_timing = input.cycle_timing;
    if (!configured_) {
        output.status = "DELAY_AUGMENTED_SOLVER_NOT_CONFIGURED";
        return false;
    }
    if (reference.empty()) {
        output.status = "NO_REFERENCE_PATH";
        return false;
    }
    if (!input.execution_horizon.active ||
        !input.phase_rejoin.active || !input.phase_rejoin.enforce ||
        !input.phase_rejoin.delay_augmented.active) {
        output.status = "DELAY_AUGMENTED_TYPED_CONTEXT_REQUIRED";
        return false;
    }
    if (input.phase_rejoin.delay_augmented.recovery_artifact_hash !=
            params_.delay_augmented_phase
                .expected_recovery_artifact_hash) {
        output.status = "DELAY_AUGMENTED_RECOVERY_HASH_MISMATCH";
        return false;
    }
    if (!weightsMatchRuntimeConfig(
            input.phase_rejoin.delay_augmented.weights,
            variant_, params_.slosh)) {
        output.status = "DELAY_AUGMENTED_COST_CONTRACT_MISMATCH";
        return false;
    }

    std::string error;
    if (!capsule_.create(
            input.execution_horizon,
            params_.delay_augmented_phase.required_capabilities,
            error)) {
        output.status = "DELAY_AUGMENTED_CONTEXT_REJECTED_" + error;
        return false;
    }
    const DelayAugmentedPhaseParameterMatrix parameters =
        DelayAugmentedPhaseParameterBuilder::build(
            input.phase_rejoin.delay_augmented);
    if (!parameters.valid) {
        output.status = "DELAY_AUGMENTED_PARAMETER_REJECTED_" +
            parameters.status;
        return false;
    }
    if (!capsule_.setParameterImage(parameters, error)) {
        output.status = "DELAY_AUGMENTED_PARAMETER_UPDATE_FAILED_" + error;
        return false;
    }
    if (!capsule_.setTerminalEmpiricalGateEnforced(
            input.phase_rejoin.delay_augmented
                .terminal_empirical_gate_enforced,
            error)) {
        output.status =
            "DELAY_AUGMENTED_EMPIRICAL_GATE_MODE_FAILED_" + error;
        return false;
    }
    std::vector<DelayAugmentedPhaseControl> nominal_controls;
    nominal_controls.reserve(manifest::kHorizonSteps);
    for (int stage = 0; stage < manifest::kHorizonSteps; ++stage) {
        const PhaseNominalStage& nominal =
            input.phase_rejoin.delay_augmented.stages[
                static_cast<std::size_t>(stage)];
        DelayAugmentedPhaseControl control;
        control.acceleration = nominal.a;
        control.angular_acceleration = nominal.alpha;
        control.progress_rate = nominal.v_s;
        nominal_controls.push_back(control);
    }
    // SQP_RTI linearizes once.  A control-only guess paired with the same
    // initial state at every shooting node can make that single QP ill posed
    // even though the nominal controls themselves are valid.  Initialize the
    // complete multiple-shooting trajectory with the frozen C++ dynamics so
    // the guess is equality-feasible and uses exactly the same causal command
    // semantics later used for independent result admission.
    if (!capsule_.setCausalWarmStart(
            input.execution_horizon, nominal_controls, error)) {
        output.status = "DELAY_AUGMENTED_WARM_START_FAILED_" + error;
        return false;
    }

    DelayAugmentedPhaseResidualDiagnostics warm_start_residuals;
    capsule_.evaluateCurrentResiduals(warm_start_residuals);
    std::vector<double> warm_start_states;
    std::vector<double> warm_start_controls;
    DelayAugmentedPhaseConstraintAudit warm_start_audit;
    if (capsule_.captureTrajectory(
            warm_start_states, warm_start_controls)) {
        warm_start_audit = DelayAugmentedPhaseAcadosSolver::auditTrajectory(
            input.execution_horizon, parameters,
            warm_start_states, warm_start_controls,
            input.phase_rejoin.delay_augmented
                .terminal_empirical_gate_enforced);
    }
    if (!warm_start_audit.evaluated) {
        output.status = "DELAY_AUGMENTED_WARM_START_AUDIT_FAILED_" +
            warm_start_audit.status;
        return false;
    }
    // A full SQP correctness backend may legitimately start from a primal
    // guess that violates an inequality and repair it.  The warm-start audit
    // is evidence, not the final admission gate.  Causal inconsistency is
    // different: it means the supplied multiple-shooting trajectory does not
    // represent the frozen 22D dynamics and must still fail closed before the
    // optimizer is invoked.
    if (warm_start_audit.max_causal_state_error >
            manifest::kMaxCausalStateError) {
        output.status =
            "DELAY_AUGMENTED_WARM_START_CAUSAL_CONSISTENCY_FAILED";
        return false;
    }

    const int status = capsule_.solve();
    output.solver_time_ms = capsule_.solveTimeSec() * 1000.0;
    output.pre_solve_snapshot.valid = true;
    output.pre_solve_snapshot.backend =
        kSolverBackendDelayAugmentedPhaseAcados;
    output.pre_solve_snapshot.solver_id = manifest::kSolverId;
    output.pre_solve_snapshot.nlp_solver_type = manifest::kNlpSolverType;
    output.pre_solve_snapshot.solver_config_hash =
        manifest::kSolverConfigHash;
    output.pre_solve_snapshot.variant = variant_.name;
    output.pre_solve_snapshot.slosh_enabled = true;
    output.pre_solve_snapshot.control_semantics =
        "published_command_rate";
    output.pre_solve_snapshot.dt = input.execution_horizon.contract.dt;
    output.pre_solve_snapshot.horizon_steps = manifest::kHorizonSteps;
    output.pre_solve_snapshot.state_width = manifest::kStateCount;
    output.pre_solve_snapshot.control_width = manifest::kControlCount;
    output.pre_solve_snapshot.parameter_width = manifest::kParameterCount;
    output.pre_solve_snapshot.parameter_names = parameters.parameter_names;
    output.pre_solve_snapshot.stage_parameters = parameters.values;
    output.pre_solve_snapshot.warm_start_residuals =
        warm_start_residuals;
    output.pre_solve_snapshot.warm_start_constraint_audit =
        warm_start_audit;
    const DelayAugmentedPhaseSolveDiagnostics& diagnostics =
        capsule_.lastSolveDiagnostics();
    output.pre_solve_snapshot.solver_residuals_evaluated =
        diagnostics.evaluated;
    output.pre_solve_snapshot.solver_nlp_status = diagnostics.nlp_status;
    output.pre_solve_snapshot.solver_qp_status = diagnostics.qp_status;
    output.pre_solve_snapshot.stationarity_residual =
        diagnostics.stationarity_residual;
    output.pre_solve_snapshot.equality_residual =
        diagnostics.equality_residual;
    output.pre_solve_snapshot.inequality_residual =
        diagnostics.inequality_residual;
    output.pre_solve_snapshot.complementarity_residual =
        diagnostics.complementarity_residual;
    output.pre_solve_snapshot.solver_sqp_iterations =
        diagnostics.sqp_iterations;
    output.pre_solve_snapshot.solver_qp_iterations =
        diagnostics.qp_iterations;
    output.pre_solve_snapshot.solver_step_length = diagnostics.step_length;
    output.pre_solve_snapshot.solver_cost = diagnostics.cost;
    output.pre_solve_snapshot.acados_solve_time_ms = output.solver_time_ms;
    output.pre_solve_snapshot.solver_iterations = diagnostics.iterations;

    std::vector<double> raw_solution_states;
    std::vector<double> raw_solution_controls;
    DelayAugmentedPhaseConstraintAudit solution_audit;
    if (capsule_.captureTrajectory(
            raw_solution_states, raw_solution_controls)) {
        solution_audit = DelayAugmentedPhaseAcadosSolver::auditTrajectory(
            input.execution_horizon, parameters,
            raw_solution_states, raw_solution_controls,
            input.phase_rejoin.delay_augmented
                .terminal_empirical_gate_enforced);
    }
    output.pre_solve_snapshot.solution_constraint_audit = solution_audit;
    if (status != 0) {
        output.failure_kind = diagnostics.optimizer_invoked
            ? SolverFailureKind::Optimization
            : SolverFailureKind::Integrity;
        output.status = "DELAY_AUGMENTED_ACADOS_SOLVE_FAILED_" +
            std::to_string(status) + "_" + diagnostics.status;
        output.pre_solve_snapshot.failed_raw_solution_states =
            std::move(raw_solution_states);
        output.pre_solve_snapshot.failed_raw_solution_controls =
            std::move(raw_solution_controls);
        output.pre_solve_snapshot.solver_status = output.status;
        return false;
    }
    if (!solution_audit.evaluated || !solution_audit.passed) {
        output.failure_kind = SolverFailureKind::Integrity;
        output.status =
            "DELAY_AUGMENTED_INDEPENDENT_CONSTRAINT_AUDIT_FAILED_" +
            solution_audit.status;
        output.pre_solve_snapshot.failed_raw_solution_states =
            std::move(raw_solution_states);
        output.pre_solve_snapshot.failed_raw_solution_controls =
            std::move(raw_solution_controls);
        output.pre_solve_snapshot.solver_status = output.status;
        return false;
    }

    DelayAugmentedPhaseRolloutResult causal_rollout;
    if (!capsule_.causalRollout(
            input.execution_horizon, causal_rollout, error)) {
        output.status = "DELAY_AUGMENTED_CAUSAL_AUDIT_FAILED_" + error;
        output.pre_solve_snapshot.solver_status = output.status;
        return false;
    }
    if (!phaseResidualsAdmissible(
            input.phase_rejoin.delay_augmented,
            causal_rollout, error)) {
        output.status = "DELAY_AUGMENTED_CAUSAL_CONSTRAINT_FAILED_" + error;
        output.pre_solve_snapshot.solver_status = output.status;
        return false;
    }

    const DelayAugmentedPhaseControl& first_control =
        causal_rollout.controls.front();
    output.cmd_v = causal_rollout.published_commands.front().linear;
    output.cmd_omega = causal_rollout.published_commands.front().angular;
    if (!std::isfinite(output.cmd_v) || !std::isfinite(output.cmd_omega) ||
        output.cmd_v < manifest::kLinearOutputMin - 1e-9 ||
        output.cmd_v > manifest::kLinearOutputMax + 1e-9 ||
        output.cmd_omega < manifest::kAngularOutputMin - 1e-9 ||
        output.cmd_omega > manifest::kAngularOutputMax + 1e-9) {
        output.status = "DELAY_AUGMENTED_FIRST_COMMAND_INVALID";
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
        return false;
    }

    output.predicted_horizon.valid = true;
    output.predicted_horizon.backend =
        kSolverBackendDelayAugmentedPhaseAcados;
    output.predicted_horizon.variant = variant_.name;
    output.predicted_horizon.solver_status = "OK";
    output.predicted_horizon.slosh_enabled = true;
    output.predicted_horizon.control_semantics =
        "published_command_rate";
    output.predicted_horizon.dt = manifest::kDt;
    output.predicted_horizon.states.reserve(manifest::kHorizonSteps + 1);
    output.predicted_horizon.controls.reserve(manifest::kHorizonSteps);
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        const DelayAugmentedPhaseState& state = causal_rollout.states[
            static_cast<std::size_t>(stage)];
        output.predicted_horizon.states.push_back(
            horizonState(state));
        TrajectoryPoint point;
        point.x = state.execution.robot.x;
        point.y = state.execution.robot.y;
        point.yaw = state.execution.robot.yaw;
        point.v = state.execution.robot.v;
        point.s = state.progress_s;
        output.trajectory.push_back(point);
        if (stage == 0) {
            output.initial_execution_state = state.execution;
            // SolverOutput::progress_abs_s is the progress of the state at
            // which this cycle was solved.  SpmpcProblem uses it as the
            // monotonic lower bound for the next projection, so publishing
            // the predicted terminal progress here would move that bound one
            // whole horizon ahead on every cycle.
            output.progress_abs_s = state.progress_s;
        }
        if (stage == manifest::kHorizonSteps) {
            output.terminal_execution_state = state.execution;
        }
        if (stage < manifest::kHorizonSteps) {
            const DelayAugmentedPhaseControl& control =
                causal_rollout.controls[static_cast<std::size_t>(stage)];
            HorizonControlDebug debug;
            debug.a = control.acceleration;
            debug.alpha_or_omega = control.angular_acceleration;
            debug.v_s = control.progress_rate;
            output.predicted_horizon.controls.push_back(debug);
        }
    }
    output.delay_augmented_execution_solution =
        output.initial_execution_state.valid &&
        output.terminal_execution_state.valid;
    output.progress_s = reference.length() > 1e-9
        ? output.progress_abs_s / reference.length()
        : 0.0;

    const RobotState& origin = input.execution_horizon.initial_state.robot;
    ProgressProjector projector;
    const auto raw_projection = projector.project(
        reference, origin.x, origin.y);
    const auto guarded_projection = projector.project(
        reference, origin.x, origin.y, input.min_progress_s);
    output.projector_debug.raw_valid = raw_projection.valid;
    output.projector_debug.raw_s = raw_projection.s;
    output.projector_debug.raw_distance = raw_projection.distance;
    output.projector_debug.guarded_valid = guarded_projection.valid;
    output.projector_debug.guarded_s = guarded_projection.s;
    output.projector_debug.guarded_distance = guarded_projection.distance;

    output.first_shot_debug.success = true;
    output.first_shot_debug.status_code = 0.0;
    output.first_shot_debug.progress_abs_s =
        input.execution_horizon.initial_progress_s;
    output.first_shot_debug.progress_s = reference.length() > 1e-9
        ? input.execution_horizon.initial_progress_s / reference.length()
        : 0.0;
    output.first_shot_debug.x0_v =
        input.execution_horizon.initial_state.robot.v;
    output.first_shot_debug.x0_omega =
        input.execution_horizon.initial_state.robot.omega;
    output.first_shot_debug.x0_s =
        input.execution_horizon.initial_progress_s;
    output.first_shot_debug.u0_a = first_control.acceleration;
    output.first_shot_debug.u0_alpha =
        first_control.angular_acceleration;
    output.first_shot_debug.u0_v_s = first_control.progress_rate;
    output.first_shot_debug.cmd_v_pre_clamp = output.cmd_v;
    output.first_shot_debug.cmd_v_post_clamp = output.cmd_v;
    output.first_shot_debug.cmd_omega_pre_clamp = output.cmd_omega;
    output.first_shot_debug.cmd_omega_post_clamp = output.cmd_omega;
    if (output.predicted_horizon.states.size() > 1) {
        const HorizonStateDebug& first = output.predicted_horizon.states[1];
        output.first_shot_debug.x1_v = first.v;
        output.first_shot_debug.x1_omega = first.omega;
        output.first_shot_debug.x1_s = first.s;
    }
    output.success = true;
    output.failure_kind = SolverFailureKind::None;
    output.status = "DELAY_AUGMENTED_ACADOS_OK";
    output.pre_solve_snapshot.solver_status = output.status;
    return true;
}

}  // namespace spmpc_local_planner
