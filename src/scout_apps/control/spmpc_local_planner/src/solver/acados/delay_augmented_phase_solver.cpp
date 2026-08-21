#include "spmpc_local_planner/solver/acados/delay_augmented_phase_solver.h"

#include "spmpc_delay_augmented_phase_solver_manifest.h"

#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_spmpc_delay_augmented_phase.h"
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace spmpc_local_planner {
namespace {

namespace manifest = delay_augmented_phase_solver_manifest;

static_assert(
    manifest::kDiscreteDynamics == DELAY_AUGMENTED_DISCRETE_DYNAMICS,
    "generated discrete-dynamics capability bit drifted");
static_assert(
    manifest::kAugmentedInitialState == DELAY_AUGMENTED_INITIAL_STATE,
    "generated initial-state capability bit drifted");
static_assert(
    manifest::kPublishedCommandBounds ==
        DELAY_AUGMENTED_PUBLISHED_COMMAND_BOUNDS,
    "generated published-command capability bit drifted");
static_assert(
    manifest::kRobotSpeedBounds == DELAY_AUGMENTED_ROBOT_SPEED_BOUNDS,
    "generated speed capability bit drifted");
static_assert(
    manifest::kPublishedRateBounds ==
        DELAY_AUGMENTED_PUBLISHED_RATE_BOUNDS,
    "generated rate capability bit drifted");
static_assert(
    manifest::kTerminalEmpiricalGate ==
        DELAY_AUGMENTED_TERMINAL_EMPIRICAL_GATE,
    "generated terminal-gate capability bit drifted");
static_assert(
    manifest::kExecutionCompatibilitySet ==
        DELAY_AUGMENTED_EXECUTION_COMPATIBILITY_SET,
    "generated execution-set capability bit drifted");
static_assert(
    manifest::kPublishedResidualBounds ==
        DELAY_AUGMENTED_PUBLISHED_RESIDUAL_BOUNDS,
    "generated residual-bound capability bit drifted");
static_assert(
    manifest::kCapabilities == kDelayAugmentedPhaseFormalCapabilities,
    "generated delay-augmented capability mask drifted");
static_assert(
    manifest::kFormalRequiredCapabilities ==
        kDelayAugmentedPhaseFormalCapabilities,
    "generated formal capability mask drifted");

bool sameChannel(const ExecutionChannelContract& channel,
                 bool linear) {
    return channel.delay_sec ==
               (linear ? manifest::kLinearDelaySec
                       : manifest::kAngularDelaySec) &&
        channel.time_constant_sec ==
               (linear ? manifest::kLinearTimeConstantSec
                       : manifest::kAngularTimeConstantSec) &&
        channel.positive_gain ==
               (linear ? manifest::kLinearPositiveGain
                       : manifest::kAngularPositiveGain) &&
        channel.negative_gain ==
               (linear ? manifest::kLinearNegativeGain
                       : manifest::kAngularNegativeGain) &&
        channel.deadzone ==
               (linear ? manifest::kLinearDeadzone
                       : manifest::kAngularDeadzone) &&
        channel.output_min ==
               (linear ? manifest::kLinearOutputMin
                       : manifest::kAngularOutputMin) &&
        channel.output_max ==
               (linear ? manifest::kLinearOutputMax
                       : manifest::kAngularOutputMax) &&
        channel.integer_delay_steps ==
               (linear ? manifest::kLinearIntegerDelaySteps
                       : manifest::kAngularIntegerDelaySteps) &&
        channel.fractional_delay_sec ==
               (linear ? manifest::kLinearFractionalDelaySec
                       : manifest::kAngularFractionalDelaySec);
}

bool finiteInRange(double value, double lower, double upper) {
    return std::isfinite(value) && value >= lower && value <= upper;
}

bool validInitialState(const ExecutionHorizonContext& context) {
    const ExecutionAugmentedState& state = context.initial_state;
    if (!state.valid || !std::isfinite(context.initial_progress_s) ||
        context.initial_progress_s < 0.0 ||
        !std::isfinite(state.robot.x) || !std::isfinite(state.robot.y) ||
        !std::isfinite(state.robot.yaw) ||
        !finiteInRange(
            state.robot.v,
            manifest::kLinearOutputMin,
            manifest::kLinearOutputMax) ||
        !finiteInRange(
            state.robot.omega,
            manifest::kAngularOutputMin,
            manifest::kAngularOutputMax) ||
        !std::isfinite(state.slosh.eta_x) ||
        !std::isfinite(state.slosh.eta_x_dot) ||
        !std::isfinite(state.slosh.eta_y) ||
        !std::isfinite(state.slosh.eta_y_dot) ||
        state.robot.v != state.linear.actuator_output ||
        state.robot.omega != state.angular.actuator_output ||
        state.linear.pending_commands.size() !=
            static_cast<std::size_t>(manifest::kLinearBufferCount) ||
        state.angular.pending_commands.size() !=
            static_cast<std::size_t>(manifest::kAngularBufferCount)) {
        return false;
    }
    for (double value : state.linear.pending_commands) {
        if (!finiteInRange(
                value,
                manifest::kLinearOutputMin,
                manifest::kLinearOutputMax)) {
            return false;
        }
    }
    for (double value : state.angular.pending_commands) {
        if (!finiteInRange(
                value,
                manifest::kAngularOutputMin,
                manifest::kAngularOutputMax)) {
            return false;
        }
    }
    return true;
}

std::array<double, manifest::kStateCount> serializeInitialState(
    const ExecutionHorizonContext& context) {
    std::array<double, manifest::kStateCount> state{};
    state[0] = context.initial_state.robot.x;
    state[1] = context.initial_state.robot.y;
    state[2] = context.initial_state.robot.yaw;
    state[3] = context.initial_state.robot.v;
    state[4] = context.initial_progress_s;
    state[5] = context.initial_state.robot.omega;
    state[6] = context.initial_state.slosh.eta_x;
    state[7] = context.initial_state.slosh.eta_x_dot;
    state[8] = context.initial_state.slosh.eta_y;
    state[9] = context.initial_state.slosh.eta_y_dot;
    for (int index = 0; index < manifest::kLinearBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kLinearBufferOffset + index)] =
                context.initial_state.linear.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    for (int index = 0; index < manifest::kAngularBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kAngularBufferOffset + index)] =
                context.initial_state.angular.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    return state;
}

std::array<double, manifest::kStateCount> serializeState(
    const DelayAugmentedPhaseState& phase_state) {
    std::array<double, manifest::kStateCount> state{};
    state[0] = phase_state.execution.robot.x;
    state[1] = phase_state.execution.robot.y;
    state[2] = phase_state.execution.robot.yaw;
    state[3] = phase_state.execution.robot.v;
    state[4] = phase_state.progress_s;
    state[5] = phase_state.execution.robot.omega;
    state[6] = phase_state.execution.slosh.eta_x;
    state[7] = phase_state.execution.slosh.eta_x_dot;
    state[8] = phase_state.execution.slosh.eta_y;
    state[9] = phase_state.execution.slosh.eta_y_dot;
    for (int index = 0; index < manifest::kLinearBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kLinearBufferOffset + index)] =
                phase_state.execution.linear.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    for (int index = 0; index < manifest::kAngularBufferCount; ++index) {
        state[static_cast<std::size_t>(
            manifest::kAngularBufferOffset + index)] =
                phase_state.execution.angular.pending_commands[
                    static_cast<std::size_t>(index)];
    }
    return state;
}

bool validDecodedState(
    const std::array<double, manifest::kStateCount>& state) {
    if (!std::all_of(state.begin(), state.end(), [](double value) {
            return std::isfinite(value);
        }) || state[4] < 0.0 ||
        !finiteInRange(state[3], manifest::kLinearOutputMin,
                       manifest::kLinearOutputMax) ||
        !finiteInRange(state[5], manifest::kAngularOutputMin,
                       manifest::kAngularOutputMax)) {
        return false;
    }
    for (int index = 0; index < manifest::kLinearBufferCount; ++index) {
        if (!finiteInRange(
                state[static_cast<std::size_t>(
                    manifest::kLinearBufferOffset + index)],
                manifest::kLinearOutputMin,
                manifest::kLinearOutputMax)) {
            return false;
        }
    }
    for (int index = 0; index < manifest::kAngularBufferCount; ++index) {
        if (!finiteInRange(
                state[static_cast<std::size_t>(
                    manifest::kAngularBufferOffset + index)],
                manifest::kAngularOutputMin,
                manifest::kAngularOutputMax)) {
            return false;
        }
    }
    return true;
}

}  // namespace

struct DelayAugmentedPhaseAcadosSolver::Impl {
    DelayAugmentedPhaseSolveDiagnostics diagnostics;
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    spmpc_delay_augmented_phase_solver_capsule* capsule = nullptr;
#endif
};

DelayAugmentedPhaseAcadosSolver::DelayAugmentedPhaseAcadosSolver()
    : impl_(new Impl()) {}

DelayAugmentedPhaseAcadosSolver::~DelayAugmentedPhaseAcadosSolver() {
    reset();
}

std::uint32_t DelayAugmentedPhaseAcadosSolver::compiledCapabilities() {
    return manifest::kCapabilities;
}

DelayAugmentedPhaseCompiledContract
DelayAugmentedPhaseAcadosSolver::compiledContract() {
    DelayAugmentedPhaseCompiledContract compiled;
    compiled.execution.schema_version =
        manifest::kExecutionContractSchemaVersion;
    compiled.execution.contract_id = manifest::kContractId;
    compiled.execution.contract_hash = manifest::kContractHash;
    compiled.execution.dt = manifest::kDt;
    compiled.execution.linear.delay_sec = manifest::kLinearDelaySec;
    compiled.execution.linear.time_constant_sec =
        manifest::kLinearTimeConstantSec;
    compiled.execution.linear.positive_gain = manifest::kLinearPositiveGain;
    compiled.execution.linear.negative_gain = manifest::kLinearNegativeGain;
    compiled.execution.linear.deadzone = manifest::kLinearDeadzone;
    compiled.execution.linear.output_min = manifest::kLinearOutputMin;
    compiled.execution.linear.output_max = manifest::kLinearOutputMax;
    compiled.execution.linear.integer_delay_steps =
        manifest::kLinearIntegerDelaySteps;
    compiled.execution.linear.fractional_delay_sec =
        manifest::kLinearFractionalDelaySec;
    compiled.execution.angular.delay_sec = manifest::kAngularDelaySec;
    compiled.execution.angular.time_constant_sec =
        manifest::kAngularTimeConstantSec;
    compiled.execution.angular.positive_gain =
        manifest::kAngularPositiveGain;
    compiled.execution.angular.negative_gain =
        manifest::kAngularNegativeGain;
    compiled.execution.angular.deadzone = manifest::kAngularDeadzone;
    compiled.execution.angular.output_min = manifest::kAngularOutputMin;
    compiled.execution.angular.output_max = manifest::kAngularOutputMax;
    compiled.execution.angular.integer_delay_steps =
        manifest::kAngularIntegerDelaySteps;
    compiled.execution.angular.fractional_delay_sec =
        manifest::kAngularFractionalDelaySec;
    compiled.slosh.dt = manifest::kDt;
    compiled.slosh.container_radius = manifest::kContainerRadius;
    compiled.slosh.liquid_height = manifest::kLiquidHeight;
    compiled.slosh.liquid_density = manifest::kLiquidDensity;
    compiled.slosh.damping_ratio = manifest::kDampingRatio;
    compiled.slosh.mode_index = manifest::kModeIndex;
    compiled.slosh.slosh_height_ref = manifest::kSloshHeightRef;
    compiled.slosh.slosh_eta_dot_ratio = manifest::kSloshEtaDotRatio;
    compiled.slosh.use_linear_model = manifest::kUseLinearModel;
    compiled.slosh.use_parabola_term = manifest::kUseParabolaTerm;
    compiled.state_width = manifest::kStateCount;
    compiled.control_width = manifest::kControlCount;
    compiled.horizon_steps = manifest::kHorizonSteps;
    compiled.execution_front_steps = manifest::kExecutionFrontSteps;
    compiled.liquid_horizon_steps = manifest::kLiquidHorizonSteps;
    compiled.parameter_schema_version = manifest::kParameterSchemaVersion;
    compiled.parameter_schema_id = manifest::kParameterSchemaId;
    compiled.parameter_schema_hash = manifest::kParameterSchemaHash;
    compiled.execution_compatibility_contract =
        manifest::kExecutionCompatibilityContract;
    compiled.capabilities = manifest::kCapabilities;
    compiled.acceleration_max = manifest::kAccelerationMax;
    compiled.angular_acceleration_max =
        manifest::kAngularAccelerationMax;
    compiled.progress_rate_max = manifest::kProgressRateMax;
    return compiled;
}

bool DelayAugmentedPhaseAcadosSolver::validateContextContract(
    const ExecutionHorizonContext& context,
    std::uint32_t required_capabilities,
    std::string& error) {
    error.clear();
    if (required_capabilities == 0u ||
        (manifest::kCapabilities & required_capabilities) !=
            required_capabilities) {
        error = "delay-augmented solver capability mismatch";
        return false;
    }
    if (!context.active ||
        context.contract.schema_version !=
            manifest::kExecutionContractSchemaVersion ||
        context.contract.contract_id != manifest::kContractId ||
        context.contract.contract_hash != manifest::kContractHash ||
        context.contract.dt != manifest::kDt ||
        !sameChannel(context.contract.linear, true) ||
        !sameChannel(context.contract.angular, false)) {
        error = "delay-augmented solver execution contract mismatch";
        return false;
    }
    if (context.execution_front_steps != manifest::kExecutionFrontSteps ||
        context.liquid_horizon_steps != manifest::kLiquidHorizonSteps ||
        context.horizon_steps != manifest::kHorizonSteps ||
        !validStamp(context.initial_epoch_ns) ||
        !validInitialState(context)) {
        error = "delay-augmented solver context shape mismatch";
        return false;
    }
    const StampNs physical_front = addSeconds(
        context.initial_epoch_ns,
        std::max(
            manifest::kLinearDelaySec,
            manifest::kAngularDelaySec));
    const StampNs grid_front = addSeconds(
        context.initial_epoch_ns,
        manifest::kExecutionFrontSteps * manifest::kDt);
    const StampNs terminal = addSeconds(
        context.initial_epoch_ns,
        manifest::kHorizonSteps * manifest::kDt);
    if (!validStamp(physical_front) || !validStamp(grid_front) ||
        !validStamp(terminal) ||
        context.physical_front_epoch_ns != physical_front ||
        context.grid_front_epoch_ns != grid_front ||
        context.terminal_epoch_ns != terminal) {
        error = "delay-augmented solver epoch mismatch";
        return false;
    }
    return true;
}

bool DelayAugmentedPhaseAcadosSolver::compiled() {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    return true;
#else
    return false;
#endif
}

void DelayAugmentedPhaseAcadosSolver::reset() {
    impl_->diagnostics = DelayAugmentedPhaseSolveDiagnostics{};
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (impl_->capsule != nullptr) {
        spmpc_delay_augmented_phase_acados_free(impl_->capsule);
        spmpc_delay_augmented_phase_acados_free_capsule(impl_->capsule);
        impl_->capsule = nullptr;
    }
#endif
}

bool DelayAugmentedPhaseAcadosSolver::create(
    const ExecutionHorizonContext& context,
    std::uint32_t required_capabilities,
    std::string& error) {
    reset();
    if (!validateContextContract(
            context, required_capabilities, error)) {
        return false;
    }
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NX == manifest::kStateCount,
        "generated capsule state width drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NU == manifest::kControlCount,
        "generated capsule control width drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_N == manifest::kHorizonSteps,
        "generated capsule horizon drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NP == manifest::kParameterCount,
        "generated capsule parameter image drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NBX == manifest::kStateBoundCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NBX0 ==
            manifest::kInitialStateBoundCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NBXN ==
            manifest::kTerminalStateBoundCount,
        "generated capsule state-bound contract drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NBU ==
            manifest::kControlBoundCount,
        "generated capsule control-bound contract drifted");
    static_assert(
        SPMPC_DELAY_AUGMENTED_PHASE_NH ==
            manifest::kPublishedCommandConstraintCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NH0 ==
            manifest::kPublishedCommandConstraintCount &&
        SPMPC_DELAY_AUGMENTED_PHASE_NHN ==
            manifest::kTerminalRecoveryConstraintCount,
        "generated capsule terminal recovery constraint drifted");

    impl_->capsule =
        spmpc_delay_augmented_phase_acados_create_capsule();
    if (impl_->capsule == nullptr ||
        spmpc_delay_augmented_phase_acados_create(impl_->capsule) != 0) {
        if (impl_->capsule != nullptr) {
            spmpc_delay_augmented_phase_acados_free_capsule(impl_->capsule);
            impl_->capsule = nullptr;
        }
        error = "failed to create delay-augmented acados capsule";
        return false;
    }

    const auto initial = serializeInitialState(context);
    ocp_nlp_config* config =
        spmpc_delay_augmented_phase_acados_get_nlp_config(impl_->capsule);
    ocp_nlp_dims* dims =
        spmpc_delay_augmented_phase_acados_get_nlp_dims(impl_->capsule);
    ocp_nlp_in* input =
        spmpc_delay_augmented_phase_acados_get_nlp_in(impl_->capsule);
    ocp_nlp_out* output =
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule);
    ocp_nlp_constraints_model_set(
        config, dims, input, output, 0, "lbx",
        const_cast<double*>(initial.data()));
    ocp_nlp_constraints_model_set(
        config, dims, input, output, 0, "ubx",
        const_cast<double*>(initial.data()));
    const std::array<double, manifest::kControlCount> zero_control{};
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        ocp_nlp_out_set(
            config, dims, output, input, stage, "x",
            const_cast<double*>(initial.data()));
        if (stage < manifest::kHorizonSteps) {
            ocp_nlp_out_set(
                config, dims, output, input, stage, "u",
                const_cast<double*>(zero_control.data()));
        }
    }
    return true;
#else
    error = "delay-augmented acados capsule is not compiled";
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::setParameterImage(
    const DelayAugmentedPhaseParameterMatrix& parameters,
    std::string& error) {
    error.clear();
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || !parameters.hasCanonicalShape()) {
        error = "delay-augmented parameter image shape mismatch";
        return false;
    }
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        const double* data = parameters.stageData(stage);
        if (data == nullptr ||
            spmpc_delay_augmented_phase_acados_update_params(
                impl_->capsule,
                stage,
                const_cast<double*>(data),
                manifest::kParameterCount) != 0) {
            error = "failed to update delay-augmented parameters at stage " +
                std::to_string(stage);
            return false;
        }
    }
    return true;
#else
    (void)parameters;
    error = "delay-augmented acados capsule is not compiled";
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::ready() const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    return impl_->capsule != nullptr;
#else
    return false;
#endif
}

int DelayAugmentedPhaseAcadosSolver::stateWidth() const {
    return ready() ? manifest::kStateCount : 0;
}

int DelayAugmentedPhaseAcadosSolver::controlWidth() const {
    return ready() ? manifest::kControlCount : 0;
}

int DelayAugmentedPhaseAcadosSolver::horizonSteps() const {
    return ready() ? manifest::kHorizonSteps : 0;
}

bool DelayAugmentedPhaseAcadosSolver::setControlGuess(
    int stage, const double* control) {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || control == nullptr || stage < 0 ||
        stage >= manifest::kHorizonSteps) {
        return false;
    }
    ocp_nlp_out_set(
        spmpc_delay_augmented_phase_acados_get_nlp_config(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_dims(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_in(impl_->capsule),
        stage, "u", const_cast<double*>(control));
    return true;
#else
    (void)stage;
    (void)control;
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::setCausalWarmStart(
    const ExecutionHorizonContext& context,
    const std::vector<DelayAugmentedPhaseControl>& controls,
    std::string& error) {
    error.clear();
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || controls.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps)) {
        error = "invalid causal warm-start shape";
        return false;
    }
    if (!validateContextContract(
            context, kDelayAugmentedPhaseFormalCapabilities, error)) {
        return false;
    }
    for (std::size_t stage = 0; stage < controls.size(); ++stage) {
        const DelayAugmentedPhaseControl& control = controls[stage];
        if (!finiteInRange(control.acceleration,
                           -manifest::kAccelerationMax,
                           manifest::kAccelerationMax) ||
            !finiteInRange(control.angular_acceleration,
                           -manifest::kAngularAccelerationMax,
                           manifest::kAngularAccelerationMax) ||
            !finiteInRange(control.progress_rate, 0.0,
                           manifest::kProgressRateMax)) {
            error = "causal warm-start control out of bounds at stage " +
                std::to_string(stage);
            return false;
        }
    }

    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    DelayAugmentedPhaseDynamics dynamics;
    if (!dynamics.configure(compiled.execution, compiled.slosh, error)) {
        error = "causal warm-start model rejected: " + error;
        return false;
    }
    const DelayAugmentedPhaseRolloutResult rollout =
        dynamics.rollout(context, controls);
    if (!rollout.valid || rollout.states.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps + 1)) {
        error = "causal warm-start rollout failed: " + rollout.status;
        return false;
    }

    ocp_nlp_config* config =
        spmpc_delay_augmented_phase_acados_get_nlp_config(impl_->capsule);
    ocp_nlp_dims* dims =
        spmpc_delay_augmented_phase_acados_get_nlp_dims(impl_->capsule);
    ocp_nlp_in* input =
        spmpc_delay_augmented_phase_acados_get_nlp_in(impl_->capsule);
    ocp_nlp_out* output =
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule);
    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        const auto state = serializeState(
            rollout.states[static_cast<std::size_t>(stage)]);
        ocp_nlp_out_set(
            config, dims, output, input, stage, "x",
            const_cast<double*>(state.data()));
        if (stage < manifest::kHorizonSteps) {
            const DelayAugmentedPhaseControl& control = controls[
                static_cast<std::size_t>(stage)];
            const std::array<double, manifest::kControlCount> raw = {{
                control.acceleration,
                control.angular_acceleration,
                control.progress_rate,
            }};
            ocp_nlp_out_set(
                config, dims, output, input, stage, "u",
                const_cast<double*>(raw.data()));
        }
    }
    return true;
#else
    (void)context;
    (void)controls;
    error = "delay-augmented acados capsule is not compiled";
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::getState(
    int stage, double* state) const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || state == nullptr || stage < 0 ||
        stage > manifest::kHorizonSteps) {
        return false;
    }
    ocp_nlp_out_get(
        spmpc_delay_augmented_phase_acados_get_nlp_config(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_dims(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule),
        stage, "x", state);
    return true;
#else
    (void)stage;
    (void)state;
    return false;
#endif
}

bool DelayAugmentedPhaseAcadosSolver::getControl(
    int stage, double* control) const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready() || control == nullptr || stage < 0 ||
        stage >= manifest::kHorizonSteps) {
        return false;
    }
    ocp_nlp_out_get(
        spmpc_delay_augmented_phase_acados_get_nlp_config(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_dims(impl_->capsule),
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule),
        stage, "u", control);
    return true;
#else
    (void)stage;
    (void)control;
    return false;
#endif
}

int DelayAugmentedPhaseAcadosSolver::solve() {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    impl_->diagnostics = DelayAugmentedPhaseSolveDiagnostics{};
    if (!ready()) {
        impl_->diagnostics.status = "CAPSULE_NOT_READY";
        return -1;
    }
    impl_->diagnostics.optimizer_invoked = true;
    const int status =
        spmpc_delay_augmented_phase_acados_solve(impl_->capsule);
    impl_->diagnostics.nlp_status = status;
    if (status != 0) {
        impl_->diagnostics.status = "NLP_STATUS_" +
            std::to_string(status);
        return status;
    }

    ocp_nlp_solver* solver =
        spmpc_delay_augmented_phase_acados_get_nlp_solver(impl_->capsule);
    ocp_nlp_in* input =
        spmpc_delay_augmented_phase_acados_get_nlp_in(impl_->capsule);
    ocp_nlp_out* output =
        spmpc_delay_augmented_phase_acados_get_nlp_out(impl_->capsule);
    ocp_nlp_eval_residuals(solver, input, output);
    ocp_nlp_get(solver, "qp_status", &impl_->diagnostics.qp_status);
    ocp_nlp_get(solver, "res_stat",
                &impl_->diagnostics.stationarity_residual);
    ocp_nlp_get(solver, "res_eq",
                &impl_->diagnostics.equality_residual);
    ocp_nlp_get(solver, "res_ineq",
                &impl_->diagnostics.inequality_residual);
    ocp_nlp_get(solver, "res_comp",
                &impl_->diagnostics.complementarity_residual);
    impl_->diagnostics.evaluated = true;
    impl_->diagnostics.residual_admitted =
        residualsAdmissible(impl_->diagnostics);
    impl_->diagnostics.status = impl_->diagnostics.residual_admitted
        ? "OK" : "RESIDUAL_REJECTED";
    return impl_->diagnostics.residual_admitted ? 0 : -2;
#else
    impl_->diagnostics = DelayAugmentedPhaseSolveDiagnostics{};
    impl_->diagnostics.status = "CAPSULE_NOT_COMPILED";
    return -1;
#endif
}

const DelayAugmentedPhaseSolveDiagnostics&
DelayAugmentedPhaseAcadosSolver::lastSolveDiagnostics() const {
    return impl_->diagnostics;
}

bool DelayAugmentedPhaseAcadosSolver::residualsAdmissible(
    const DelayAugmentedPhaseSolveDiagnostics& diagnostics) {
    return diagnostics.evaluated && diagnostics.nlp_status == 0 &&
        diagnostics.qp_status == 0 &&
        std::isfinite(diagnostics.stationarity_residual) &&
        std::isfinite(diagnostics.equality_residual) &&
        std::isfinite(diagnostics.inequality_residual) &&
        std::isfinite(diagnostics.complementarity_residual) &&
        diagnostics.equality_residual <=
            manifest::kMaxEqualityResidual &&
        diagnostics.inequality_residual <=
            manifest::kMaxInequalityResidual;
}

bool DelayAugmentedPhaseAcadosSolver::causalRollout(
    const ExecutionHorizonContext& context,
    DelayAugmentedPhaseRolloutResult& rollout,
    std::string& error) const {
    rollout = DelayAugmentedPhaseRolloutResult{};
    error.clear();
    if (!ready()) {
        error = "delay-augmented capsule is not ready";
        return false;
    }
    if (!validateContextContract(
            context, kDelayAugmentedPhaseFormalCapabilities, error)) {
        return false;
    }

    std::vector<DelayAugmentedPhaseControl> controls;
    controls.reserve(static_cast<std::size_t>(manifest::kHorizonSteps));
    for (int stage = 0; stage < manifest::kHorizonSteps; ++stage) {
        std::array<double, manifest::kControlCount> raw{};
        if (!getControl(stage, raw.data()) ||
            !finiteInRange(raw[0], -manifest::kAccelerationMax,
                           manifest::kAccelerationMax) ||
            !finiteInRange(raw[1], -manifest::kAngularAccelerationMax,
                           manifest::kAngularAccelerationMax) ||
            !finiteInRange(raw[2], 0.0,
                           manifest::kProgressRateMax)) {
            error = "invalid decoded control at stage " +
                std::to_string(stage);
            return false;
        }
        DelayAugmentedPhaseControl control;
        control.acceleration = raw[0];
        control.angular_acceleration = raw[1];
        control.progress_rate = raw[2];
        controls.push_back(control);
    }

    const DelayAugmentedPhaseCompiledContract compiled = compiledContract();
    DelayAugmentedPhaseDynamics dynamics;
    if (!dynamics.configure(compiled.execution, compiled.slosh, error)) {
        error = "causal audit model rejected: " + error;
        return false;
    }
    rollout = dynamics.rollout(context, controls);
    if (!rollout.valid || rollout.states.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps + 1) ||
        rollout.published_commands.size() !=
            static_cast<std::size_t>(manifest::kHorizonSteps)) {
        error = "causal rollout failed: " + rollout.status;
        return false;
    }

    for (int stage = 0; stage <= manifest::kHorizonSteps; ++stage) {
        std::array<double, manifest::kStateCount> decoded{};
        if (!getState(stage, decoded.data()) ||
            !validDecodedState(decoded)) {
            error = "invalid decoded state at stage " +
                std::to_string(stage);
            return false;
        }
        const auto causal = serializeState(
            rollout.states[static_cast<std::size_t>(stage)]);
        for (std::size_t index = 0; index < decoded.size(); ++index) {
            if (std::abs(decoded[index] - causal[index]) >
                    manifest::kMaxCausalStateError) {
                error = "causal state mismatch at stage " +
                    std::to_string(stage) + " index " +
                    std::to_string(index);
                return false;
            }
        }
    }
    for (std::size_t stage = 0;
         stage < rollout.published_commands.size(); ++stage) {
        const VelocityCommand& command =
            rollout.published_commands[stage];
        if (!finiteInRange(command.linear,
                           manifest::kLinearOutputMin,
                           manifest::kLinearOutputMax) ||
            !finiteInRange(command.angular,
                           manifest::kAngularOutputMin,
                           manifest::kAngularOutputMax)) {
            error = "causal published command out of bounds at stage " +
                std::to_string(stage);
            return false;
        }
    }
    return true;
}

double DelayAugmentedPhaseAcadosSolver::solveTimeSec() const {
#ifdef SPMPC_WITH_ACADOS_DELAY_AUGMENTED_PHASE
    if (!ready()) return 0.0;
    double solve_time = 0.0;
    ocp_nlp_get(
        spmpc_delay_augmented_phase_acados_get_nlp_solver(impl_->capsule),
        "time_tot", &solve_time);
    return solve_time;
#else
    return 0.0;
#endif
}

}  // namespace spmpc_local_planner
